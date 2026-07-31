// OpenAI Realtime (WebRTC) AI assistant panel.

import { api, post } from "./api.js?v=28";
import { state, applySceneSnapshot } from "./store.js?v=36";
import { renderProgramEditor, renderTree, updateStatus, openProgramWorkspace } from "./ui.js?v=52";
import { startSimulation, clearSimulation, renderPlanPath } from "./viewport.js?v=48";

const realtime = {
  pc: null,
  dc: null,
  stream: null,
  remoteStream: null,
  connected: false,
  talking: false,
  talkStartedAt: 0,
  responseActive: false,
  responseCreatePending: false,
  responseRequested: false,
  cancelWhenCreated: false,
  outputPlaying: false,
  toolInProgress: false,
  toolCalls: new Set(),
};
// The API requires at least 100 ms of buffered audio. A 200 ms wall-clock
// guard leaves room for browser/WebRTC packet startup so an accepted press
// cannot arrive as an 80 ms buffer.
const MIN_PUSH_TO_TALK_MS = 200;
const PLANNING_TOOLS = new Set([
  "plan_home_zero",
  "plan_spatial_move",
  "plan_move_to_point",
  "plan_pick_place",
  "plan_program",
]);
const $ = (sel) => document.querySelector(sel);

function setRealtimeStatus(message) {
  const el = $("#realtimeStatus");
  if (el) el.textContent = message;
}

function appendOutput(text) {
  const el = $("#realtimeOutput");
  if (!el) return;
  el.textContent += text;
  el.scrollTop = el.scrollHeight;
}

function planSummary(plan) {
  if (!plan?.ok) return plan?.error || "Plan failed.";
  const seconds = Number(plan.durationMs || 0) / 1000;
  const lines = [
    `Plan: ${plan.program} - ${plan.steps?.length || 0} states, est ${seconds.toFixed(1)}s`,
  ];
  const spatial = plan.spatialResolution;
  const selected = spatial?.selectedPosition;
  if (selected && Number.isFinite(Number(selected.x)) && Number.isFinite(Number(selected.y))) {
    lines.push(
      `Destination: ${spatial.region || spatial.destinationKind || "resolved point"} at X ${Number(selected.x).toFixed(3)} m, Y ${Number(selected.y).toFixed(3)} m.`
    );
  }
  if (spatial?.coordinateReason) lines.push(`reason: ${spatial.coordinateReason}`);
  for (const note of plan.notes || []) lines.push(`note: ${note}`);
  if (plan.safetyGate?.reason) lines.push(plan.safetyGate.reason);
  return lines.join("\n");
}

function showPlanInProgramPanel(plan, program = null) {
  if (!plan?.ok) return;
  if (program) {
    state.activeProgramId = program.id || null;
    state.draftName = program.name || plan.program || "Voice Program";
    state.draftSteps = (program.steps || []).map((step) => ({ ...step }));
    state.draftRepeatCount = Number(program.repeatCount || plan.repeatCount || 1);
    state.programDirty = false;
  } else {
    state.activeProgramId = null;
    state.draftName = plan.program || "Voice Program";
    if (program?.steps) state.draftSteps = program.steps.map((step) => ({ ...step }));
  }
  state.lastPlan = plan;
  state.planSource = "program";
  const output = $("#planOutput");
  if (output) output.textContent = planSummary(plan);
  renderTree();
  renderProgramEditor();
  openProgramWorkspace();
  renderPlanPath(plan);
  if (!plan.requiresCapturedToolRpy) {
    startSimulation(plan);
  } else {
    clearSimulation({ preservePath: true });
    renderPlanPath(plan);
  }
}

async function syncSavedProgramResult(result) {
  if (!result.program) return;
  try {
    applySceneSnapshot(await api("/api/scene"));
  } catch {
    if (!state.programs.some((program) => program.id === result.program.id)) {
      state.programs = [...state.programs, result.program];
    }
  }
  showPlanInProgramPanel(result.plan, result.program);
  updateStatus(`Saved program ${result.program.name}.`);
}

function syncPendingRunResult(result) {
  if (!result.program || !result.plan) return;
  showPlanInProgramPanel(result.plan, result.program);
  updateStatus(`Ready to run ${result.program.name}; waiting for voice confirmation.`);
}

function setButtonState() {
  const connectBtn = $("#realtimeConnectBtn");
  const talkBtn = $("#realtimeTalkBtn");
  if (connectBtn) connectBtn.textContent = realtime.connected ? "Disconnect" : "Connect";
  if (talkBtn) {
    talkBtn.disabled = !realtime.connected;
    talkBtn.textContent = realtime.talking ? "Listening… Release to Send" : "Hold to Talk";
    talkBtn.classList.toggle("listening", realtime.talking);
  }
}

function flushResponseCreate() {
  if (!realtime.responseRequested) return;
  if (!realtime.dc || realtime.dc.readyState !== "open") return;
  if (realtime.responseActive || realtime.responseCreatePending || realtime.toolInProgress) return;
  realtime.responseRequested = false;
  realtime.responseCreatePending = true;
  realtime.dc.send(JSON.stringify({
    type: "response.create",
    response: { output_modalities: ["audio"] },
  }));
}

function requestResponseCreate() {
  realtime.responseRequested = true;
  flushResponseCreate();
}

function interruptActiveResponse() {
  if (realtime.responseActive) {
    sendRealtimeEvent({ type: "response.cancel" });
  } else if (realtime.responseCreatePending) {
    realtime.cancelWhenCreated = true;
  }
  if (realtime.outputPlaying) {
    sendRealtimeEvent({ type: "output_audio_buffer.clear" });
    realtime.outputPlaying = false;
  }
}

async function handleEvent(event) {
  if (event.type === "response.created") {
    realtime.responseCreatePending = false;
    realtime.responseActive = true;
    if (realtime.cancelWhenCreated || realtime.talking) {
      realtime.cancelWhenCreated = false;
      sendRealtimeEvent({ type: "response.cancel", response_id: event.response?.id });
    }
  }
  if (event.type === "response.output_text.delta" && event.delta) appendOutput(event.delta);
  if ((event.type === "response.audio_transcript.delta" || event.type === "response.output_audio_transcript.delta") && event.delta) {
    appendOutput(event.delta);
  }
  if (event.type === "conversation.item.input_audio_transcription.completed" && event.transcript) {
    appendOutput(`\nYou: ${event.transcript}\n`);
  }
  if (event.type === "input_audio_buffer.speech_started") setRealtimeStatus("Listening...");
  if (event.type === "input_audio_buffer.speech_stopped") setRealtimeStatus("Thinking...");
  if (event.type === "response.audio.delta" || event.type === "response.output_audio.delta") {
    realtime.outputPlaying = true;
    setRealtimeStatus("Speaking...");
  }
  if (["response.audio.done", "response.output_audio.done", "output_audio_buffer.cleared"].includes(event.type)) {
    realtime.outputPlaying = false;
  }
  if (event.type === "error") {
    const message = event.error?.message || "Realtime error";
    realtime.responseCreatePending = false;
    appendOutput(`\nError: ${message}\n`);
    setRealtimeStatus(message);
  }
  if (event.type === "response.done") {
    realtime.responseActive = false;
    realtime.responseCreatePending = false;
    realtime.outputPlaying = false;
    let sentToolOutput = false;
    const functionCalls = (event.response?.output || []).filter((entry) => entry.type === "function_call");
    realtime.toolInProgress = functionCalls.length > 0;
    try {
      for (const item of functionCalls) {
        sentToolOutput = (await runTool(item)) || sentToolOutput;
      }
    } finally {
      realtime.toolInProgress = false;
    }
    appendOutput("\n");
    if (sentToolOutput) requestResponseCreate();
    else flushResponseCreate();
    if (
      realtime.connected && !realtime.talking
      && !realtime.responseActive && !realtime.responseCreatePending && !realtime.responseRequested
    ) setRealtimeStatus("Connected. Hold to Talk.");
  }
}

async function runTool(item) {
  if (!item?.call_id || realtime.toolCalls.has(item.call_id)) return false;
  realtime.toolCalls.add(item.call_id);
  let args = {};
  let result;
  try {
    if (PLANNING_TOOLS.has(item.name)) {
      setRealtimeStatus("Planning and validating the path...");
      const output = $("#planOutput");
      if (output) output.textContent = "AI is calculating a safe destination and validating the complete path...";
    }
    args = item.arguments ? JSON.parse(item.arguments) : {};
    if (!args || Array.isArray(args) || typeof args !== "object") throw new Error("Tool arguments must be an object.");
    result = await post("/api/realtime/tool", { name: item.name, arguments: args });
    if (result.ok && result.steps) {
      const draft = args.steps ? { name: args.name || result.program, steps: args.steps } : null;
      showPlanInProgramPanel(result, draft);
    } else if (result.ok && result.plan?.steps) {
      if (item.name === "request_program_run") syncPendingRunResult(result);
      else if (result.program) await syncSavedProgramResult(result);
      else showPlanInProgramPanel(result.plan);
    }
    if (item.name === "update_virtual_layout" && result.ok) {
      try {
        applySceneSnapshot(await api("/api/scene"));
        renderTree();
        renderProgramEditor();
        updateStatus(result.warning || "Updated the simulation layout.");
      } catch {
        // The normal scene poll will recover if this one refresh fails.
      }
    }
    if (item.name === "confirm_program_run" && result.executedSteps) {
      const output = $("#planOutput");
      if (output) {
        const status = result.ok ? "Voice physical run finished." : `Voice physical run failed: ${result.error || "unknown error"}`;
        output.textContent = `${output.textContent}\n\n${status}`;
      }
      try {
        applySceneSnapshot(await api("/api/scene"));
        renderTree();
        renderProgramEditor();
      } catch {
        // Best effort: the regular polling path will recover the scene.
      }
    }
    if (PLANNING_TOOLS.has(item.name) && !result.ok) {
      const output = $("#planOutput");
      const failures = (result.candidateFailures || [])
        .slice(0, 3)
        .map((failure, index) => `Option ${index + 1}: ${failure.error || "path validation failed"}`);
      if (output) {
        output.textContent = [
          "AI plan could not be created.",
          result.error || "The requested destination did not validate.",
          ...failures,
        ].join("\n");
      }
      clearSimulation();
      updateStatus(`AI plan failed: ${result.error || "no valid path"}`);
    }
  } catch (error) {
    result = { ok: false, error: `Tool ${item.name || "call"} failed: ${error.message}` };
    appendOutput(`\n${result.error}\n`);
  }
  if (realtime.dc?.readyState === "open") {
    realtime.dc.send(JSON.stringify({
      type: "conversation.item.create",
      item: { type: "function_call_output", call_id: item.call_id, output: JSON.stringify(result) },
    }));
    return true;
  }
  return false;
}

async function connect() {
  if (realtime.connected) {
    disconnect();
    return;
  }
  if (!window.RTCPeerConnection) {
    setRealtimeStatus("WebRTC unavailable in this browser.");
    return;
  }
  setRealtimeStatus("Connecting...");
  setButtonState();
  realtime.stream = await navigator.mediaDevices.getUserMedia({
    audio: {
      echoCancellation: true,
      noiseSuppression: true,
      autoGainControl: true,
    },
  });
  realtime.pc = new RTCPeerConnection();
  for (const track of realtime.stream.getAudioTracks()) {
    // Push-to-talk is authoritative. Merely connecting never submits room audio.
    track.enabled = false;
    realtime.pc.addTrack(track, realtime.stream);
  }
  realtime.pc.addTransceiver("audio", { direction: "recvonly" });
  realtime.pc.addEventListener("track", (event) => {
    const audio = $("#realtimeAudio");
    if (!audio) return;
    const [stream] = event.streams;
    realtime.remoteStream = stream || new MediaStream([event.track]);
    audio.srcObject = realtime.remoteStream;
    audio.play().catch(() => {
      setRealtimeStatus("Connected. Click in the page if audio is blocked.");
    });
  });
  realtime.dc = realtime.pc.createDataChannel("oai-events");
  realtime.dc.addEventListener("open", () => {
    realtime.connected = true;
    setRealtimeStatus("Connected. Hold to Talk.");
    setButtonState();
  });
  realtime.dc.addEventListener("message", (event) => {
    try {
      const payload = JSON.parse(event.data);
      handleEvent(payload).catch((error) => appendOutput(`\nRealtime event error: ${error.message}\n`));
    } catch {
      appendOutput("\nRealtime event error: received malformed server data.\n");
    }
  });
  realtime.dc.addEventListener("close", () => {
    disconnect("Disconnected.");
  });
  const offer = await realtime.pc.createOffer();
  await realtime.pc.setLocalDescription(offer);
  const response = await fetch("/api/realtime/session", {
    method: "POST",
    headers: { "Content-Type": "application/sdp" },
    body: offer.sdp,
  });
  const answerSdp = await response.text();
  if (!response.ok) {
    setRealtimeStatus(answerSdp);
    throw new Error(answerSdp);
  }
  await realtime.pc.setRemoteDescription({ type: "answer", sdp: answerSdp });
}

function disconnect(message = "Disconnected.") {
  if (realtime.dc) realtime.dc.close();
  if (realtime.pc) realtime.pc.close();
  if (realtime.stream) {
    for (const track of realtime.stream.getTracks()) track.stop();
  }
  const audio = $("#realtimeAudio");
  if (audio) audio.srcObject = null;
  realtime.pc = null;
  realtime.dc = null;
  realtime.stream = null;
  realtime.remoteStream = null;
  realtime.connected = false;
  realtime.talking = false;
  realtime.talkStartedAt = 0;
  realtime.responseActive = false;
  realtime.responseCreatePending = false;
  realtime.responseRequested = false;
  realtime.cancelWhenCreated = false;
  realtime.outputPlaying = false;
  realtime.toolInProgress = false;
  realtime.toolCalls.clear();
  setRealtimeStatus(message);
  setButtonState();
}

function sendRealtimeEvent(payload) {
  if (!realtime.dc || realtime.dc.readyState !== "open") return;
  realtime.dc.send(JSON.stringify(payload));
}

function beginTalk(event) {
  if (!realtime.connected || realtime.talking) return;
  if (event?.pointerId !== undefined) event.currentTarget?.setPointerCapture?.(event.pointerId);
  realtime.talking = true;
  realtime.talkStartedAt = performance.now();
  interruptActiveResponse();
  sendRealtimeEvent({ type: "input_audio_buffer.clear" });
  for (const track of realtime.stream?.getAudioTracks?.() || []) track.enabled = true;
  setRealtimeStatus("Listening...");
  setButtonState();
}

function endTalk() {
  if (!realtime.connected || !realtime.talking) return;
  realtime.talking = false;
  for (const track of realtime.stream?.getAudioTracks?.() || []) track.enabled = false;
  const durationMs = performance.now() - realtime.talkStartedAt;
  realtime.talkStartedAt = 0;
  if (durationMs < MIN_PUSH_TO_TALK_MS) {
    sendRealtimeEvent({ type: "input_audio_buffer.clear" });
    setRealtimeStatus("Too short—hold the button while speaking.");
    setButtonState();
    return;
  }
  sendRealtimeEvent({ type: "input_audio_buffer.commit" });
  setRealtimeStatus("Thinking...");
  setButtonState();
  requestResponseCreate();
}

function sendText() {
  if (!realtime.connected || !realtime.dc || realtime.dc.readyState !== "open") {
    setRealtimeStatus("Connect first.");
    return;
  }
  const input = $("#realtimePromptInput");
  const text = input.value.trim();
  if (!text) return;
  appendOutput(`\nYou: ${text}\n`);
  input.value = "";
  interruptActiveResponse();
  realtime.dc.send(JSON.stringify({
    type: "conversation.item.create",
    item: { type: "message", role: "user", content: [{ type: "input_text", text }] },
  }));
  requestResponseCreate();
}

export async function initRealtime() {
  try {
    const payload = await api("/api/realtime/status");
    setRealtimeStatus(payload.configured ? `Ready (${payload.model}, voice ${payload.voice || "default"})` : "Set OPENAI_API_KEY and restart.");
  } catch (error) {
    setRealtimeStatus(`Status unavailable: ${error.message}`);
  }
  $("#realtimeConnectBtn").addEventListener("click", () => {
    connect().catch((error) => setRealtimeStatus(error.message));
  });
  const talkBtn = $("#realtimeTalkBtn");
  talkBtn.addEventListener("pointerdown", beginTalk);
  talkBtn.addEventListener("pointerup", endTalk);
  talkBtn.addEventListener("pointercancel", endTalk);
  talkBtn.addEventListener("lostpointercapture", endTalk);
  talkBtn.addEventListener("keydown", (event) => {
    if ((event.key === " " || event.key === "Enter") && !event.repeat) {
      event.preventDefault();
      beginTalk(event);
    }
  });
  talkBtn.addEventListener("keyup", (event) => {
    if (event.key === " " || event.key === "Enter") {
      event.preventDefault();
      endTalk();
    }
  });
  $("#realtimeSendBtn").addEventListener("click", sendText);
  $("#realtimePromptInput").addEventListener("keydown", (event) => {
    if (event.key === "Enter" && !event.shiftKey) {
      event.preventDefault();
      sendText();
    }
  });
  setButtonState();
}
