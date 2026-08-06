// Entry point: boot the workcell editor.

import { state, applySceneSnapshot, applyTagTracks, on } from "./store.js?v=37";
import { api } from "./api.js?v=28";
import { initViewport, renderEnvironment } from "./viewport.js?v=50";
import {
  initUI, renderTree, renderInspector, renderProgramEditor,
  buildJointControls, setTargetInputs, updateAngleReadouts, updateStatus, loadPorts,
} from "./ui.js?v=56";
import { initRealtime } from "./realtime.js?v=43";

const ANGLE_POLL_INTERVAL_MS = 150;
const SCENE_POLL_INTERVAL_MS = 4000;
let anglePollInFlight = false;
let tagPollInFlight = false;

function updateMeasuredAngles(angles) {
  state.angles = angles.map(Number);
  if (!state.renderInitialized) {
    state.renderAngles = [...state.angles];
    state.renderInitialized = true;
  }
  updateAngleReadouts();
  if (!state.targetsInitialized) {
    setTargetInputs(state.angles);
    state.targetsInitialized = true;
  }
}

async function pollAngles() {
  if (anglePollInFlight) return;
  anglePollInFlight = true;
  try {
    const payload = await api("/api/angles");
    state.connected = payload.ok;
    state.executing = Boolean(payload.executing);
    const executionSourceStepId = payload.executionProgress?.sourceStepId || null;
    if (executionSourceStepId !== state.executionSourceStepId) {
      state.executionSourceStepId = executionSourceStepId;
      if (state.programWorkspaceOpen) renderProgramEditor();
    }
    state.lastError = payload.ok ? null : payload.error;
    if (payload.angles) updateMeasuredAngles(payload.angles);
    if (state.executing || state.physicalRunActive) {
      const progress = payload.executionProgress;
      updateStatus(progress?.stateId
        ? `Running ${progress.stateId} - Stop aborts it.`
        : "Physical program running - Stop aborts it.");
    } else {
      updateStatus(payload.ok ? "Live: reading joint angles." : "Robot not responding.");
    }
  } catch (error) {
    state.connected = false;
    state.lastError = error.message;
    if (!state.physicalRunActive) updateStatus("Dashboard API error.");
  } finally {
    anglePollInFlight = false;
  }
}

async function pollScene() {
  if (state.dragActive || state.simulation || state.physicalRunActive || state.executing) return;
  try {
    const payload = await api("/api/scene");
    if (
      payload.version !== state.sceneVersion
      && !state.dragActive
      && !state.simulation
      && !state.physicalRunActive
      && !state.executing
    ) {
      applySceneSnapshot(payload);
    }
  } catch {
    // best-effort; angle poll reports connectivity
  }
}

async function pollTagTracks() {
  if (tagPollInFlight || state.dragActive || state.simulation || state.physicalRunActive) return;
  tagPollInFlight = true;
  try { applyTagTracks(await api(`/api/camera/tag-tracks?since=${state.tagTrackRevision || 0}`)); }
  catch { /* camera tracking may be stopped */ }
  finally { tagPollInFlight = false; }
}

async function loadStatus() {
  const payload = await api("/api/status");
  state.connected = payload.connected;
  state.lastError = payload.lastError;
  state.port = payload.port;
  if (payload.jointLimits) state.limits = payload.jointLimits;
  if (payload.lastAngles) updateMeasuredAngles(payload.lastAngles);
  if (payload.port) {
    const select = document.querySelector("#portSelect");
    if (select) select.value = payload.port;
  }
  updateStatus(payload.port ? `Using ${payload.port}` : "Pick a serial port (Robot panel).");
}

async function init() {
  buildJointControls();
  initUI();
  await initViewport(document.querySelector("#robotViewport"), updateStatus);
  await loadPorts().catch(() => {});
  await loadStatus().catch(() => {});
  applySceneSnapshot(await api("/api/scene").catch(() => null));
  renderTree();
  renderInspector();
  renderProgramEditor();
  renderEnvironment();
  await initRealtime();

  await pollAngles();
  setInterval(pollAngles, ANGLE_POLL_INTERVAL_MS);
  setInterval(pollScene, SCENE_POLL_INTERVAL_MS);
  setInterval(pollTagTracks, 100);
}

init().catch((error) => {
  state.lastError = error.message;
  updateStatus("Startup failed.");
  console.error(error);
});
