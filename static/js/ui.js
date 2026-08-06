// Cell browser tree, properties inspector, and program editor.

import { state, on, emit, setSelection, applySceneSnapshot, findPart, findBin, findPoint, clamp } from "./store.js?v=37";
import { api, post } from "./api.js?v=28";
import {
  startSimulation, clearSimulation, renderPlanPath, renderEnvironment,
  syncEndEffector, SCENE_BOUND_METERS, pauseSimulation, resumeSimulation,
  seekSimulationSource,
} from "./viewport.js?v=50";

const $ = (sel) => document.querySelector(sel);
const HOME_ANGLES = [0, 0, 0, 0, 0, -45];
let cameraFormDirty = false;
let calibrationWizardStep = 1;
let calibrationCaptureCount = 0;
let captureDiversityPassed = false;
let markerMapSaved = false;
let intrinsicsSolved = false;
let workspaceVerified = false;
let cameraPoseAccepted = false;
let accuracyVerified = false;
let calibrationTestingBypass = false;
let partWizardPartId = null;
let partWizardSelectedTagId = null;
let partWizardTagTimer = null;
let partWizardUnits = "in";
let partWizardLatestTagPayload = null;
let pointWizardDraft = null;
let viewportHome = null;
let activeJogSessionId = null;
let jogHeartbeatTimer = null;
let tcpJogTimer = null;

function showInspectorTab() {
  const tab = $("#tabInspector");
  if (tab) tab.click();
}

function setCameraFormDirty(dirty) {
  cameraFormDirty = Boolean(dirty);
}

export function updateStatus(message) {
  const errorText = state.lastError ? `  Error: ${state.lastError}` : "";
  $("#statusText").textContent = `${message}${errorText}`;
  const pill = $("#connectionPill");
  pill.textContent = state.executing ? "Running" : state.connected ? "Online" : "Offline";
  pill.classList.toggle("online", state.connected && !state.executing);
  pill.classList.toggle("running", Boolean(state.executing));
}

function renderEndEffectorSelect() {
  const select = $("#endEffectorSelect");
  if (!select) return;
  const options = state.endEffectors.length
    ? state.endEffectors
    : [
      { id: "adaptive_gripper", label: "Adaptive Gripper" },
      { id: "suction_gripper", label: "Air Suction Gripper" },
    ];
  select.innerHTML = "";
  for (const option of options) {
    const item = document.createElement("option");
    item.value = option.id;
    item.textContent = option.label;
    item.selected = option.id === state.endEffector;
    select.append(item);
  }
}

function renderGripperActionLabels() {
  const suction = state.endEffector === "suction_gripper";
  const open = $("#gripperOpenBtn");
  const auto = $("#gripperAutoBtn");
  const close = $("#gripperCloseBtn");
  if (open) open.textContent = suction ? "Suction Off" : "Grip Open";
  if (auto) auto.textContent = suction ? "Auto Suction" : "Auto Grip";
  if (close) close.textContent = suction ? "Suction On" : "Grip Close";
}

function renderToolOrientationStatus() {
  const el = $("#toolOrientationStatus");
  if (!el) return;
  const rpy = state.coordinatePlanner?.toolRpyDeg;
  if (rpy) {
    el.textContent = `Captured flange reference: ${Number(rpy.rx || 0).toFixed(1)}, ${Number(rpy.ry || 0).toFixed(1)}, ${Number(rpy.rz || 0).toFixed(1)} deg. Picks use a canonical top-down jaw pose.`;
  } else {
    el.textContent = "Picks use a modeled, canonical top-down jaw pose. Capturing the flange is optional diagnostic reference data.";
  }
}

function renderPickCalibration() {
  const planner = state.coordinatePlanner || {};
  const biasInput = $("#pickHeightBiasInput");
  const clearanceInput = $("#minimumTableClearanceInput");
  const status = $("#pickCalibrationStatus");
  if (biasInput) biasInput.value = String(Math.round(Number(planner.pickHeightBiasM || 0) * 1000));
  if (clearanceInput) clearanceInput.value = String(Math.round(Number(planner.minimumTableClearanceM || 0.004) * 1000));
  if (status) {
    status.textContent = `Modeled jaw-center transform; Pick Z Bias ${Math.round(Number(planner.pickHeightBiasM || 0) * 1000)} mm; minimum table clearance ${Math.round(Number(planner.minimumTableClearanceM || 0.004) * 1000)} mm.`;
  }
}

function renderToolContactCalibration() {
  const profile = state.coordinatePlanner?.toolProfiles?.[state.endEffector] || {};
  const correction = profile.tcpCorrectionLocalM || {};
  const geometry = profile.geometry || {};
  const status = $("#toolContactCalibrationStatus");
  if (!status) return;
  const correctionText = `saved local correction X ${Number(correction.x || 0) * 1000 >= 0 ? "+" : ""}${(Number(correction.x || 0) * 1000).toFixed(1)}, Y ${(Number(correction.y || 0) * 1000).toFixed(1)}, Z ${(Number(correction.z || 0) * 1000).toFixed(1)} mm`;
  if (state.endEffector === "suction_gripper") {
    status.textContent = `${correctionText}. Installed contact length ${(Number(geometry.flangeToContactM || 0.072) * 1000).toFixed(0)} mm; cup diameter ${(Number(geometry.cupDiameterM || 0.022) * 1000).toFixed(0)} mm. Physical center of mass is unknown.`;
  } else {
    status.textContent = `${correctionText}. The adaptive jaw-pocket CAD transform remains unchanged.`;
  }
}

function latestPickJawYawDeg() {
  const steps = state.lastPlan?.steps || [];
  const pickStep = steps.find((step) => Number.isFinite(Number(step?.desiredJawYawDeg)));
  return pickStep ? Number(pickStep.desiredJawYawDeg) : 0;
}

function cameraConfigFromInputs() {
  const selectedValue = $("#cameraDeviceSelect").value;
  if (selectedValue === "") throw new Error("Connect an external USB camera, then click Find Cameras.");
  const selectedDevice = Number(selectedValue);
  const selectedMetadata = state.cameraDevices.find((device) => Number(device.id) === selectedDevice);
  if (!selectedMetadata?.uniqueId) throw new Error("Refresh the external camera list before starting.");
  return {
    deviceId: selectedDevice,
    deviceUniqueId: selectedMetadata.uniqueId,
    deviceLabel: selectedMetadata.label,
    localization: {
      enabled: Boolean($("#continuousLocalizationInput")?.checked),
      intervalS: 0.08,
    },
  };
}

function fiducialsFromInputs() {
  return {
    dictionary: "DICT_APRILTAG_36h11",
    markerSizeM: 0.05,
    minimumMarkers: 3,
    maxConditionNumber: 1e6,
    validationProfile: "practical",
    maxReprojectionRmsPx: 10,
    maxReprojectionPx: 18,
    minimumCoverageRatio: 0.12,
    cameraMoveLimitM: 0.008,
    referenceMarkers: [0, 1, 2, 3].map((id) => ({
      id,
      sizeM: 0.05,
      center: { x: Number($(`#tag${id}X`).value), y: Number($(`#tag${id}Y`).value) },
      yawDeg: Number($(`#tag${id}Yaw`).value || 0),
    })),
  };
}

function renderCameraConfig(options = {}) {
  if (cameraFormDirty && !options.force) return;
  const config = state.camera || {};
  const deviceSelect = $("#cameraDeviceSelect");
  if (!deviceSelect) return;
  const current = config.deviceId ?? 0;
  deviceSelect.innerHTML = "";
  const devices = state.cameraDevices;
  if (!devices.length) {
    const option = document.createElement("option");
    option.value = "";
    option.textContent = "No external USB camera found";
    option.selected = true;
    deviceSelect.append(option);
  }
  for (const device of devices) {
    const option = document.createElement("option");
    option.value = String(device.id);
    option.textContent = device.width ? `${device.label} (${device.width}x${device.height})` : device.label;
    option.selected = String(device.id) === String(current);
    deviceSelect.append(option);
  }
  const fiducials = state.calibration?.fiducials || {};
  for (const marker of fiducials.referenceMarkers || []) {
    const id = Number(marker.id);
    if (id < 0 || id > 3 || !marker.center) continue;
    if ($(`#tag${id}X`)) $(`#tag${id}X`).value = Number(marker.center.x || 0);
    if ($(`#tag${id}Y`)) $(`#tag${id}Y`).value = Number(marker.center.y || 0);
    if ($(`#tag${id}Yaw`)) $(`#tag${id}Yaw`).value = Number(marker.yawDeg || 0);
  }
  if ($("#continuousLocalizationInput")) {
    $("#continuousLocalizationInput").checked = Boolean(config.localization?.enabled);
  }
}

function renderCameraStatus() {
  const status = state.cameraStatus || {};
  const config = state.camera || status.config || {};
  const calibration = state.calibration || status.calibration || {};
  const intrinsics = calibration.intrinsics || {};
  const intrinsicsPass = Boolean(intrinsics.ok) && Number(intrinsics.intrinsicRmsPx) <= 2.5 && Number(intrinsics.maximumViewErrorPx || intrinsics.intrinsicRmsPx || Infinity) <= 4;
  const poseLocked = Boolean(calibration.fiducials?.baselineHomography);
  const verificationPassed = Boolean(calibration.verification?.passed);
  const testingBypass = Boolean(calibration.verification?.testingBypass);
  const parts = state.parts.filter((part) => part.source === "camera");
  const fresh = parts.filter((part) => !part.stale).length;
  const lines = [
    status.available === false ? "OpenCV unavailable on this Python install." : status.running ? "Camera running." : "Camera stopped.",
    intrinsicsPass ? "Lens calibrated." : calibration.intrinsics ? `Lens recalibration required (RMS ${Number(intrinsics.intrinsicRmsPx || 0).toFixed(2)} px).` : "Fiducial calibration required.",
    `${fresh}/${parts.length} fresh camera detections.`,
  ];
  if (status.lastFrameSize) lines.push(`Frame ${status.lastFrameSize.width}x${status.lastFrameSize.height}, count ${status.frameCount || 0}.`);
  if (status.lastError) lines.push(`Error: ${status.lastError}`);
  $("#cameraStatus").textContent = lines.join(" ");
  const summary = $("#cameraCalibrationSummary");
  if (summary) {
    if (!intrinsicsPass) summary.textContent = "Camera calibration has not been completed.";
    else if (!poseLocked) summary.textContent = "Lens calibrated; workspace tags still need to be checked and the camera position locked.";
    else if (verificationPassed) summary.textContent = "Calibrated and verified in robot coordinates.";
    else if (testingBypass) summary.textContent = "Testing mode: camera pose is locked, but the optional nine-point accuracy check was skipped.";
    else summary.textContent = "Camera pose is locked; the optional nine-point accuracy check has not been completed.";
  }
  const preview = document.querySelector(".camera-preview");
  const img = $("#cameraPreview");
  if (status.running) {
    preview.classList.add("live");
    if (img.dataset.streaming !== "true") {
      img.src = `/api/camera/stream?t=${Date.now()}`;
      img.dataset.streaming = "true";
    }
  } else {
    preview.classList.remove("live");
    img.removeAttribute("src");
    img.dataset.streaming = "false";
  }
  if (config.deviceId !== undefined) renderCameraConfig();
}

async function refreshCameraStatus() {
  try {
    const payload = await api("/api/camera/status");
    state.cameraStatus = payload;
    state.camera = payload.config || state.camera;
    state.calibration = payload.calibration || state.calibration;
    renderCameraStatus();
  } catch (error) {
    $("#cameraStatus").textContent = `Camera status failed: ${error.message}`;
  }
}

async function refreshCameraDevices(options = {}) {
  const payload = await api("/api/camera/devices");
  state.cameraDevices = payload.devices || [];
  renderCameraConfig(options);
}

async function saveCameraConfig() {
  const payload = await post("/api/camera/config", cameraConfigFromInputs());
  if (payload.ok === false) throw new Error(payload.error || "Camera configuration was rejected.");
  setCameraFormDirty(false);
  applySceneSnapshot(payload);
  await refreshCameraStatus();
  renderCameraConfig({ force: true });
  updateStatus("Camera config saved.");
}

function renderFiducialResult(payload) {
  const quality = payload.quality || {};
  const remedies = {
    homography_poorly_conditioned: "Check X/Y signs, duplicate coordinates, ID layout, and tag yaw.",
    reprojection_error_excessive: "Check the highlighted tag measurements and print scale; edge-only errors usually mean lens distortion needs recalibration.",
    marker_inliers_insufficient: "One or more tags disagree with the others. Recheck that tag's center, 50 mm size, yaw, and flatness.",
    not_all_reference_markers_visible: "Make all four permanent tags fully visible.",
    reference_layout_invalid: "Use ID 0 forward-left, 1 forward-right, 2 rear-right, and 3 rear-left; check signs and duplicate positions.",
  };
  const markerDetails = (quality.perMarker || [])
    .map((marker) => `ID ${marker.id}: RMS ${Number(marker.rmsPx).toFixed(2)} px, max ${Number(marker.maxPx).toFixed(2)} px, ${marker.inlierCornerCount}/4 inliers${marker.passed ? "" : " FAIL"}`)
    .join(" | ");
  const metrics = quality.conditionNumber == null ? "" : ` Condition ${Number(quality.conditionNumber).toFixed(0)}; all-corner RMS ${Number(quality.allCornerRmsPx).toFixed(2)} px, max ${Number(quality.allCornerMaxPx).toFixed(2)} px; inliers ${quality.inlierCornerCount || 0}/${quality.cornerCount || 0}.`;
  const message = payload.ok
    ? `Valid: ${quality.visibleMarkerCount || 0} markers.${metrics} Coverage ${(Number(quality.coverageRatio || 0) * 100).toFixed(1)}%. ${markerDetails}`
    : `Rejected: ${payload.error || "not calibrated"}. ${quality.visibleMarkerCount || 0} marker(s) visible.${metrics} ${markerDetails} ${remedies[payload.error] || "Review the debug overlay and calibration measurements."}`;
  $("#fiducialStatus").textContent = message;
  $("#fiducialDebugPreview").src = `/api/camera/debug-frame?t=${Date.now()}`;
}

const CAPTURE_GUIDANCE = [
  "Center the board and keep it flat.", "Move it to the upper-left.", "Move it to the upper-right.",
  "Move it to the lower-left.", "Move it to the lower-right.", "Bring it closer to the camera.",
  "Move it farther from the camera.", "Tilt the left edge toward the camera.", "Tilt the right edge toward the camera.",
  "Tilt the top edge toward the camera.", "Rotate the board about 30 degrees.", "Use one final sharp, different view.",
];

function renderAccuracyRows() {
  const grid = $("#accuracyPointGrid");
  if (!grid || grid.children.length) return;
  grid.innerHTML = "<strong>#</strong><strong>Expected X</strong><strong>Expected Y</strong><strong>Camera X</strong><strong>Camera Y</strong><strong></strong>";
  for (let index = 0; index < 9; index += 1) {
    grid.insertAdjacentHTML("beforeend", `<strong>${index + 1}</strong><input data-accuracy-expected-x="${index}" type="number" step="0.001" placeholder="m"><input data-accuracy-expected-y="${index}" type="number" step="0.001" placeholder="m"><output data-accuracy-measured-x="${index}">-</output><output data-accuracy-measured-y="${index}">-</output><button type="button" data-read-accuracy="${index}">Read Camera</button>`);
  }
}

function renderCalibrationWizard() {
  document.querySelectorAll("[data-calibration-step]").forEach((section) => {
    section.hidden = Number(section.dataset.calibrationStep) !== calibrationWizardStep;
  });
  document.querySelectorAll("[data-wizard-dot]").forEach((dot) => {
    const number = Number(dot.dataset.wizardDot);
    dot.classList.toggle("active", number === calibrationWizardStep);
    dot.classList.toggle("complete", number < calibrationWizardStep);
  });
  $("#calibrationBackBtn").disabled = calibrationWizardStep === 1;
  $("#calibrationNextBtn").hidden = calibrationWizardStep === 6;
  const hints = ["Verify both print dimensions before continuing.", "Save the four measured tag centers.", "All 12 accepted photos must be different.", "Complete all three checks from left to right.", "All nine points must pass the accuracy limits.", "Calibration is ready."];
  $("#calibrationStepHint").textContent = hints[calibrationWizardStep - 1];
  const completionMessage = $("#cameraCalibrationCompleteMessage");
  if (completionMessage) {
    completionMessage.textContent = calibrationTestingBypass
      ? "Testing mode is active. The nine-point accuracy check is skipped; physical runs remain available with an explicit unverified-coordinate warning."
      : "The camera can now report table objects in robot-base coordinates. Keep the camera, tags, table, and robot base fixed.";
  }
  if (calibrationWizardStep === 3) {
    $("#calibrationLivePreview").src = `/api/camera/stream?t=${Date.now()}`;
  }
  renderAccuracyRows();
}

function calibrationStepComplete() {
  if (calibrationWizardStep === 1) return Boolean($("#printsMeasuredCheck").checked);
  if (calibrationWizardStep === 2) return markerMapSaved;
  // Let the operator reach Solve after 12 accepted photos. The solver remains
  // the authority on diversity and explains a missing view directly.
  if (calibrationWizardStep === 3) return calibrationCaptureCount >= 12;
  if (calibrationWizardStep === 4) return intrinsicsSolved && workspaceVerified && cameraPoseAccepted;
  if (calibrationWizardStep === 5) return accuracyVerified;
  return true;
}

function updateCaptureProgress(count, diversity = null) {
  calibrationCaptureCount = Number(count || 0);
  captureDiversityPassed = Boolean(diversity?.passed);
  $("#captureCount").textContent = `${calibrationCaptureCount} / 12`;
  $("#captureMeterFill").style.width = `${Math.min(100, calibrationCaptureCount / 12 * 100)}%`;
  $("#charucoCaptureBtn").textContent = calibrationCaptureCount >= 12 ? "Add Another Photo" : `Take Photo ${calibrationCaptureCount + 1}`;
  $("#charucoCaptureBtn").disabled = false;
  const missing = diversity?.missing || [];
  const friendlyMissing = {
    center: "take one photo near the image center",
    upperLeft: "move the board to the upper-left",
    upperRight: "move the board to the upper-right",
    lowerLeft: "move the board to the lower-left",
    lowerRight: "move the board to the lower-right",
    second_board_scale: "take one clearly closer or farther photo",
    four_tilted_views: "take more photos with the board visibly tilted",
  };
  const missingGuidance = missing.map((item) => friendlyMissing[item] || item.replaceAll("_", " ")).join("; ");
  $("#captureGuidance").textContent = calibrationCaptureCount >= 12
    ? (captureDiversityPassed ? "Required position, scale, and tilt diversity is complete." : `You can continue to Solve, or improve the set: ${missingGuidance}.`)
    : CAPTURE_GUIDANCE[Math.min(calibrationCaptureCount, CAPTURE_GUIDANCE.length - 1)];
}

async function readAccuracyPoint(index) {
  const payload = await post("/api/camera/calibration/verify", {});
  if (!payload.ok) throw new Error(payload.error || "Current frame is not valid.");
  const targets = (payload.visibleTags || []).filter((item) => !item.bound && item.robotTablePosition);
  if (targets.length !== 1) throw new Error(`Show exactly one unassigned object tag (ID 10–25); camera currently sees ${targets.length}.`);
  const position = targets[0].robotTablePosition;
  const xOut = $(`[data-accuracy-measured-x="${index}"]`);
  const yOut = $(`[data-accuracy-measured-y="${index}"]`);
  xOut.value = Number(position.x).toFixed(4); yOut.value = Number(position.y).toFixed(4);
  xOut.dataset.value = position.x; yOut.dataset.value = position.y;
}

async function measureStationarySpread() {
  const positions = [];
  let tagId = null;
  for (let index = 0; index < 5; index += 1) {
    const payload = await post("/api/camera/calibration/verify", {});
    if (!payload.ok) throw new Error(payload.error || "Stationary verification frame is invalid.");
    const targets = (payload.visibleTags || []).filter((item) => !item.bound && item.robotTablePosition);
    if (targets.length !== 1) throw new Error(`Keep exactly one unassigned object tag visible; camera sees ${targets.length}.`);
    if (tagId !== null && targets[0].tagId !== tagId) throw new Error("The calibration tag changed during the stationary check.");
    tagId = targets[0].tagId;
    positions.push(targets[0].robotTablePosition);
    if (index < 4) await new Promise((resolve) => setTimeout(resolve, 180));
  }
  let spread = 0;
  for (const left of positions) for (const right of positions) {
    spread = Math.max(spread, Math.hypot(Number(left.x) - Number(right.x), Number(left.y) - Number(right.y)));
  }
  return spread;
}

// ------------------------------------------------------------------ tree

function treeItem(label, meta, selected, onClick, swatch) {
  const item = document.createElement("button");
  item.type = "button";
  item.className = "tree-item" + (selected ? " selected" : "");
  if (swatch) {
    const dot = document.createElement("span");
    dot.className = "swatch";
    dot.style.background = swatch;
    item.append(dot);
  }
  const text = document.createElement("span");
  text.className = "tree-label";
  text.textContent = label;
  item.append(text);
  if (meta) {
    const tag = document.createElement("span");
    tag.className = "tree-meta";
    tag.textContent = meta;
    item.append(tag);
  }
  item.addEventListener("click", onClick);
  return item;
}

export function renderTree() {
  const robotItem = $("#treeRobot");
  robotItem.classList.toggle("selected", state.selection.kind === "robot");

  const partList = $("#partList");
  partList.innerHTML = "";
  for (const part of state.parts) {
    partList.append(treeItem(
      part.label,
      part.source === "camera" ? "cam" : "",
      state.selection.kind === "part" && state.selection.id === part.id,
      () => {
        showInspectorTab();
        setSelection("part", part.id);
      },
      part.color
    ));
  }
  const visibleIds = new Set(state.parts.map((part) => part.id));
  for (const definition of state.registeredParts.filter((item) => !visibleIds.has(item.partId))) {
    partList.append(treeItem(definition.label, `tag ${definition.tagId} · hidden`, false, () => openPartWizard(null, definition), definition.color));
  }
  if (!state.parts.length && !state.registeredParts.length) partList.innerHTML = '<div class="tree-empty">No parts yet</div>';

  const binList = $("#binList");
  binList.innerHTML = "";
  for (const bin of state.bins) {
    binList.append(treeItem(
      bin.label, "",
      state.selection.kind === "bin" && state.selection.id === bin.id,
      () => {
        showInspectorTab();
        setSelection("bin", bin.id);
      },
      bin.color
    ));
  }
  if (!state.bins.length) binList.innerHTML = '<div class="tree-empty">No bins yet</div>';

  const surfaceList = $("#surfaceList");
  surfaceList.innerHTML = "";
  for (const surface of state.supportSurfaces) {
    surfaceList.append(treeItem(
      surface.name,
      surface.id === "surface-table" ? "0 mm · fixed" : `${Math.round(Number(surface.topZ || 0) * 1000)} mm`,
      false,
      () => openSupportSurfaceDialog(surface),
      surface.color,
    ));
  }
  if (!state.supportSurfaces.length) surfaceList.innerHTML = '<div class="tree-empty">Main table only</div>';

  const pointList = $("#pointList");
  pointList.innerHTML = "";
  for (const point of state.taughtPoints) {
    pointList.append(treeItem(
      point.label,
      point.endEffector === "suction_gripper" ? "suction" : "gripper",
      state.selection.kind === "point" && state.selection.id === point.id,
      () => {
        showInspectorTab();
        setSelection("point", point.id);
      },
      "#2563eb",
    ));
  }
  if (!state.taughtPoints.length) pointList.innerHTML = '<div class="tree-empty">No taught points</div>';

  const programList = $("#programList");
  programList.innerHTML = "";
  for (const program of state.programs) {
    programList.append(treeItem(
      program.name, `${program.steps.length}`,
      state.activeProgramId === program.id,
      () => loadProgram(program.id)
    ));
  }
  if (!state.programs.length) programList.innerHTML = '<div class="tree-empty">No programs yet</div>';
}

function openSupportSurfaceDialog(surface = null) {
  const locked = surface?.id === "surface-table";
  $("#supportSurfaceId").value = surface?.id || "";
  $("#supportSurfaceName").value = surface?.name || "Platform";
  $("#supportSurfaceX").value = Math.round(Number(surface?.center?.x ?? 0.18) * 1000);
  $("#supportSurfaceY").value = Math.round(Number(surface?.center?.y ?? 0) * 1000);
  $("#supportSurfaceSizeX").value = Math.round(Number(surface?.size?.x ?? 0.20) * 1000);
  $("#supportSurfaceSizeY").value = Math.round(Number(surface?.size?.y ?? 0.20) * 1000);
  $("#supportSurfaceTopZ").value = Math.round(Number(surface?.topZ ?? 0.05) * 1000);
  $("#supportSurfaceTolerance").value = Math.round(Number(surface?.entryToleranceM ?? 0.015) * 1000);
  $("#supportSurfaceEnabled").checked = surface?.enabled !== false;
  $("#supportSurfaceForm").querySelectorAll("input").forEach((input) => { input.disabled = locked && input.id !== "supportSurfaceId"; });
  $("#deleteSupportSurfaceBtn").hidden = !surface || locked;
  $("#supportSurfaceForm").querySelector('button[type="submit"]').hidden = locked;
  $("#supportSurfaceDialog").showModal();
}

async function saveSupportSurface(event) {
  event.preventDefault();
  const tolerance = clamp(Number($("#supportSurfaceTolerance").value) / 1000, 0.015, 0.05);
  const payload = await post("/api/scene/support-surface", {
    id: $("#supportSurfaceId").value || undefined,
    name: $("#supportSurfaceName").value.trim() || "Platform",
    center: { x: Number($("#supportSurfaceX").value) / 1000, y: Number($("#supportSurfaceY").value) / 1000 },
    size: { x: Number($("#supportSurfaceSizeX").value) / 1000, y: Number($("#supportSurfaceSizeY").value) / 1000 },
    topZ: Number($("#supportSurfaceTopZ").value) / 1000,
    entryToleranceM: tolerance,
    holdToleranceM: Math.max(tolerance, 0.020),
    enabled: $("#supportSurfaceEnabled").checked,
  });
  if (!payload.ok) throw new Error(payload.error || "Surface could not be saved.");
  applySceneSnapshot(payload);
  $("#supportSurfaceDialog").close();
  renderTree();
  updateStatus(`Saved ${payload.supportSurface?.name || "support surface"}.`);
}

// -------------------------------------------------------------- inspector

function fieldRow(labelText, input) {
  const row = document.createElement("div");
  row.className = "field-row";
  const label = document.createElement("label");
  label.textContent = labelText;
  row.append(label, input);
  return row;
}

function numberInput(value, step, oninput) {
  const input = document.createElement("input");
  input.type = "number";
  input.step = String(step);
  input.value = Number(value).toFixed(step < 0.01 ? 3 : step < 1 ? 3 : 0);
  input.addEventListener("change", () => oninput(Number(input.value)));
  return input;
}

function section(titleText, open = true) {
  const details = document.createElement("details");
  details.className = "inspector-section";
  details.open = open;
  const summary = document.createElement("summary");
  summary.textContent = titleText;
  const body = document.createElement("div");
  body.className = "inspector-section-body";
  details.append(summary, body);
  return { details, body };
}

function numberField(labelText, value, step, fieldName, oninput) {
  const input = numberInput(value, step, oninput);
  input.dataset.field = fieldName;
  return fieldRow(labelText, input);
}

function textField(labelText, value, oninput) {
  const input = document.createElement("input");
  input.type = "text";
  input.value = value;
  input.addEventListener("change", () => oninput(input.value));
  return fieldRow(labelText, input);
}

function selectField(labelText, value, options, oninput) {
  const input = document.createElement("select");
  for (const [optionValue, optionLabel] of options) {
    const option = document.createElement("option");
    option.value = optionValue;
    option.textContent = optionLabel;
    option.selected = value === optionValue;
    input.append(option);
  }
  input.addEventListener("change", () => oninput(input.value));
  return fieldRow(labelText, input);
}

function colorField(labelText, value, fallback, oninput) {
  const input = document.createElement("input");
  input.type = "color";
  input.value = /^#/.test(value || "") ? value : fallback;
  input.addEventListener("change", () => oninput(input.value));
  return fieldRow(labelText, input);
}

function checkboxField(labelText, value, oninput) {
  const input = document.createElement("input");
  input.type = "checkbox";
  input.checked = Boolean(value);
  input.addEventListener("change", () => oninput(input.checked));
  return fieldRow(labelText, input);
}

function measurementFactor(units = $("#partWizardUnits")?.value || "in") {
  return units === "mm" ? 0.001 : 0.0254;
}

function selectPartWizardTag(tag) {
  if (!tag) return;
  partWizardSelectedTagId = Number(tag.tagId);
  $("#partTagSelectionStatus").textContent = tag.bound
    ? `Selected tag ${tag.tagId}, currently assigned to ${tag.label || tag.partId}. Saving will ask before reassignment.`
    : `Selected tag ${tag.tagId}.`;
  $("#partWizardResult").textContent = `AprilTag ${tag.tagId} selected. Complete the part details, then click Save Part.`;
  if (partWizardLatestTagPayload) renderPartTagOverlay(partWizardLatestTagPayload);
}

function renderPartTagChoices(payload) {
  const choices = $("#partVisibleTagChoices");
  if (!choices) return;
  choices.innerHTML = "";
  const tags = [...(payload.tags || [])].sort((left, right) => Number(left.tagId) - Number(right.tagId));
  if (!tags.length) {
    const empty = document.createElement("span");
    empty.className = "helper-text";
    empty.textContent = payload.ok
      ? "No object tags are visible. Hold or place a printed tag ID 10–25 in view."
      : `Tag selection unavailable: ${payload.error || "the localization frame is invalid"}.`;
    choices.append(empty);
    return;
  }
  for (const tag of tags) {
    const button = document.createElement("button");
    button.type = "button";
    button.className = "visible-tag-choice";
    button.dataset.tagId = String(tag.tagId);
    button.classList.toggle("bound", Boolean(tag.bound));
    button.classList.toggle("selected", Number(tag.tagId) === Number(partWizardSelectedTagId));
    const name = document.createElement("span");
    name.textContent = `AprilTag ${tag.tagId}`;
    const stateText = document.createElement("span");
    stateText.className = "tag-binding-state";
    const status = tag.localizationStatus;
    const progress = tag.stabilizationProgress || {};
    stateText.textContent = tag.bound
      ? status === "stabilizing"
        ? `Assigned; collecting ${progress.frames || 0}/${progress.required || 3} surface frames`
        : status === "rejected"
          ? `Assigned; ${friendlyTagLocalizationMessage(tag)}`
          : `Assigned to ${tag.label || tag.partId}`
      : "Available";
    button.append(name, stateText);
    // Selection happens on pointer-down so fast overlay refreshes cannot
    // replace the button between mouse-down and click.
    button.addEventListener("pointerdown", (event) => {
      event.preventDefault();
      selectPartWizardTag(tag);
    });
    button.addEventListener("click", () => selectPartWizardTag(tag));
    choices.append(button);
  }
}

function friendlyTagLocalizationMessage(tag) {
  const reason = tag.rejectionReason || tag.reason || "pose unavailable";
  const residualMm = Number(tag.nearestSurfaceResidualM) * 1000;
  if (reason === "support_surface_stabilizing") {
    const progress = tag.stabilizationProgress || {};
    return `Tag ${tag.tagId} detected; collecting ${progress.frames || 0}/${progress.required || 3} platform frames.`;
  }
  if (reason === "support_surface_ambiguous") return `Tag ${tag.tagId} matches multiple surfaces; adjust their heights or footprints.`;
  if (reason === "support_surface_outside_footprint") {
    const distanceMm = Number(tag.surfaceFootprintDistanceM) * 1000;
    return Number.isFinite(distanceMm)
      ? `Tag ${tag.tagId} matches the surface height but is ${distanceMm.toFixed(0)} mm outside its footprint; check the surface center and size.`
      : `Tag ${tag.tagId} matches a surface height but is outside its footprint; check the surface center and size.`;
  }
  if (reason === "support_surface_unknown" && Number.isFinite(residualMm)) {
    return `Tag ${tag.tagId} is ${residualMm.toFixed(0)} mm from the nearest surface height; check the platform or box height.`;
  }
  if (reason === "object_tag_geometry_inconsistent") return `Tag ${tag.tagId} is detected but does not look flat; check glare, wrinkles, and print scale.`;
  if (reason === "object_tag_size_inconsistent" || reason === "object_tag_scale_inconsistent") return `Tag ${tag.tagId} size does not match 30 mm; check print scale.`;
  return `Tag ${tag.tagId} detected; ${String(reason).replaceAll("_", " ")}.`;
}

function renderPartTagOverlay(payload) {
  partWizardLatestTagPayload = payload;
  renderPartTagChoices(payload);
  const cameraMoved = payload.error === "camera_moved_reaccept_required";
  const poseWarning = $("#partTagPoseWarning");
  if (poseWarning) poseWarning.hidden = !cameraMoved;
  const svg = $("#partTagOverlay");
  const frame = payload.frameSize;
  if (!svg) return;
  svg.innerHTML = "";
  if (!frame?.width || !frame?.height) {
    if (!payload.ok) $("#partTagSelectionStatus").textContent = `Tag view unavailable: ${payload.error || "camera frame invalid"}`;
    return;
  }
  svg.setAttribute("viewBox", `0 0 ${frame.width} ${frame.height}`);
  for (const tag of payload.tags || []) {
    const polygon = document.createElementNS("http://www.w3.org/2000/svg", "polygon");
    polygon.setAttribute("points", (tag.cornersPx || []).map((point) => `${point[0]},${point[1]}`).join(" "));
    polygon.classList.toggle("bound", Boolean(tag.bound));
    polygon.classList.toggle("localized", tag.localizationStatus === "localized");
    polygon.classList.toggle("stabilizing", tag.localizationStatus === "stabilizing");
    polygon.classList.toggle("rejected", ["rejected", "frame_invalid"].includes(tag.localizationStatus));
    polygon.classList.toggle("selected", Number(tag.tagId) === Number(partWizardSelectedTagId));
    polygon.addEventListener("pointerdown", (event) => {
      event.preventDefault();
      selectPartWizardTag(tag);
    });
    polygon.addEventListener("click", () => selectPartWizardTag(tag));
    const label = document.createElementNS("http://www.w3.org/2000/svg", "text");
    label.setAttribute("x", String(tag.centerPx?.[0] || 0));
    label.setAttribute("y", String(tag.centerPx?.[1] || 0));
    label.textContent = `ID ${tag.tagId}`;
    svg.append(polygon, label);
  }
  if (!payload.ok) {
    const selected = (payload.tags || []).find((tag) => Number(tag.tagId) === Number(partWizardSelectedTagId));
    if (selected) {
      $("#partTagSelectionStatus").textContent = `AprilTag ${selected.tagId} selected. Coordinates remain paused until the camera is relocked.`;
    } else if (cameraMoved && (payload.tags || []).length) {
      $("#partTagSelectionStatus").textContent = "Select a tag below; only robot coordinates are paused.";
    } else if (cameraMoved) {
      $("#partTagSelectionStatus").textContent = "Camera moved. Show an object tag ID 10–25 to select it, then relock for coordinates.";
    } else {
      $("#partTagSelectionStatus").textContent = `Coordinates unavailable: ${payload.error || "camera frame invalid"}`;
    }
  } else {
    const selected = (payload.tags || []).find((tag) => Number(tag.tagId) === Number(partWizardSelectedTagId));
    if (selected?.bound && selected.localizationStatus !== "localized") {
      $("#partTagSelectionStatus").textContent = friendlyTagLocalizationMessage(selected);
    }
  }
}

async function refreshPartTagPicker() {
  if (!$("#partWizardDialog")?.open || $("#partTrackingMode").value !== "apriltag") return;
  try {
    renderPartTagOverlay(await api("/api/camera/tags/visible"));
    // The selectable corners are measured in the undistorted localization
    // frame, so display that same frame instead of the raw camera stream.
    $("#partTagCamera").src = `/api/camera/debug-frame?t=${Date.now()}`;
  }
  catch (error) { $("#partTagSelectionStatus").textContent = `Tag picker unavailable: ${error.message}`; }
}

async function openPartWizard(part = null, definition = null) {
  definition ||= state.registeredParts.find((item) => item.partId === part?.id) || null;
  partWizardPartId = definition?.partId || part?.id || null;
  partWizardSelectedTagId = definition?.tagId ?? null;
  partWizardLatestTagPayload = null;
  partWizardUnits = "in";
  const source = definition || part || {};
  const size = source.size || { x: 0.0508, y: 0.0508, z: 0.0508 };
  $("#partWizardTitle").textContent = source.label ? `Configure ${source.label}` : "Add a Part";
  $("#partTrackingMode").value = definition ? "apriltag" : part ? (part.trackingMode === "apriltag" ? "apriltag" : "virtual") : "apriltag";
  $("#partWizardName").value = source.label || `Part ${state.parts.length + state.registeredParts.length + 1}`;
  $("#partWizardShape").value = source.type || "box";
  $("#partWizardUnits").value = "in";
  $("#partWizardLength").value = (Number(size.x) / 0.0254).toFixed(2);
  $("#partWizardWidth").value = (Number(size.y) / 0.0254).toFixed(2);
  $("#partWizardHeight").value = (Number(size.z) / 0.0254).toFixed(2);
  $("#partWizardColor").value = source.color || "#8a63d2";
  $("#partWizardGraspable").checked = source.graspable !== false;
  $("#partTagOffsetX").value = (Number(definition?.tagOffsetM?.x || 0) / 0.0254).toFixed(2);
  $("#partTagOffsetY").value = (Number(definition?.tagOffsetM?.y || 0) / 0.0254).toFixed(2);
  const savedYaw = Number(definition?.yawOffsetDeg || 0);
  const presetYaws = [0, 90, 180, -90];
  $("#partTagYaw").value = presetYaws.includes(savedYaw) ? String(savedYaw) : "custom";
  $("#partTagYawCustom").value = String(savedYaw);
  $("#partTagYawCustom").hidden = $("#partTagYaw").value !== "custom";
  $("#partTagPickerSection").hidden = $("#partTrackingMode").value !== "apriltag";
  $("#partTagPlacement").hidden = $("#partTrackingMode").value !== "apriltag";
  $("#partWizardResult").textContent = definition ? `Currently assigned to tag ${definition.tagId}.` : "Select a visible object tag, then enter the measured dimensions.";
  $("#partVisibleTagChoices").innerHTML = '<span class="helper-text">Looking for object tags 10–25…</span>';
  $("#partWizardDialog").showModal();
  if ($("#partTrackingMode").value === "apriltag") {
    try { await post("/api/camera/start", {}); } catch { /* status appears in picker */ }
    clearInterval(partWizardTagTimer);
    partWizardTagTimer = setInterval(refreshPartTagPicker, 150);
    refreshPartTagPicker();
  }
}

function closePartWizard() {
  clearInterval(partWizardTagTimer);
  partWizardTagTimer = null;
  $("#partTagCamera").removeAttribute("src");
  $("#partWizardDialog").close();
}

async function savePartWizard() {
  const factor = measurementFactor();
  const measurements = [
    Number($("#partWizardLength").value),
    Number($("#partWizardWidth").value),
    Number($("#partWizardHeight").value),
  ];
  if (!measurements.every((value) => Number.isFinite(value) && value > 0)) {
    throw new Error("Length, width, and height must all be positive numbers.");
  }
  const size = {
    x: clamp(measurements[0] * factor, 0.008, 0.2),
    y: clamp(measurements[1] * factor, 0.008, 0.2),
    z: clamp(measurements[2] * factor, 0.008, 0.2),
  };
  const common = {
    partId: partWizardPartId, label: $("#partWizardName").value.trim() || "Part",
    type: $("#partWizardShape").value, size, color: $("#partWizardColor").value,
    graspable: $("#partWizardGraspable").checked,
  };
  let payload;
  if ($("#partTrackingMode").value === "apriltag") {
    if (!partWizardSelectedTagId) throw new Error("Click an AprilTag ID 10–25 in the camera first.");
    const request = {
      ...common, tagId: partWizardSelectedTagId,
      tagOffsetM: { x: Number($("#partTagOffsetX").value || 0) * factor, y: Number($("#partTagOffsetY").value || 0) * factor },
      yawOffsetDeg: $("#partTagYaw").value === "custom" ? Number($("#partTagYawCustom").value || 0) : Number($("#partTagYaw").value || 0),
    };
    payload = await post("/api/scene/part/tag-binding", request);
    if (payload.requiresReassign) {
      if (!window.confirm(`${payload.error} Reassign it to this part?`)) return;
      payload = await post("/api/scene/part/tag-binding", { ...request, reassign: true });
    }
  } else if (state.registeredParts.some((item) => item.partId === partWizardPartId)) {
    payload = await post("/api/scene/part/tag-unbind", { partId: partWizardPartId });
    payload = await post("/api/scene/part", { ...common, id: partWizardPartId, position: payload.part?.position });
  } else {
    const existing = findPart(partWizardPartId);
    payload = await post("/api/scene/part", {
      ...common, id: partWizardPartId,
      position: existing?.position || { x: 0.16, y: 0.08, z: size.z / 2 }, trackingMode: "virtual",
    });
  }
  if (payload.ok === false) throw new Error(payload.error || "Part setup was rejected.");
  applySceneSnapshot(payload);
  closePartWizard();
  updateStatus(`Saved ${common.label}.`);
}

async function savePart(part) {
  try {
    applySceneSnapshot(await post("/api/scene/part", part));
    updateStatus(`Saved ${part.label}.`);
  } catch (error) {
    updateStatus(`Save failed: ${error.message}`);
  }
}

async function saveBin(bin) {
  try {
    applySceneSnapshot(await post("/api/scene/bin", bin));
    updateStatus(`Saved ${bin.label}.`);
  } catch (error) {
    updateStatus(`Save failed: ${error.message}`);
  }
}

function openPointWizard() {
  pointWizardDraft = null;
  $("#pointWizardName").value = `Point ${state.taughtPoints.length + 1}`;
  $("#pointWizardSurfaceZ").value = "0";
  $("#pointWizardUseDestination").checked = true;
  $("#pointWizardSaveBtn").disabled = true;
  $("#pointWizardReadout").textContent = "No pose captured yet. The active tool and its calibration are saved with the point.";
  $("#pointWizardDialog").showModal();
}

function closePointWizard() {
  pointWizardDraft = null;
  $("#pointWizardDialog").close();
}

async function capturePointWizardPose() {
  const payload = await post("/api/robot/points/capture", {
    label: $("#pointWizardName").value.trim() || `Point ${state.taughtPoints.length + 1}`,
    supportSurfaceZ: Number($("#pointWizardSurfaceZ").value || 0) / 1000,
    uses: $("#pointWizardUseDestination").checked ? ["waypoint", "destination"] : ["waypoint"],
    persist: false,
  });
  if (!payload.ok || !payload.pointDraft) throw new Error(payload.error || "Robot pose capture failed.");
  pointWizardDraft = payload.pointDraft;
  const point = pointWizardDraft;
  const p = point.tcpPoseM.position;
  $("#pointWizardReadout").textContent =
    `Captured ${point.endEffector}: TCP ${(p.x * 1000).toFixed(1)}, ${(p.y * 1000).toFixed(1)}, ${(p.z * 1000).toFixed(1)} mm; ` +
    `joints ${point.jointAnglesDeg.map((value) => Number(value).toFixed(1)).join(", ")}°. No motion was commanded.`;
  $("#pointWizardSaveBtn").disabled = false;
}

async function savePointWizard() {
  if (!pointWizardDraft) throw new Error("Capture the stationary robot pose first.");
  const payload = await post("/api/scene/point", {
    ...pointWizardDraft,
    label: $("#pointWizardName").value.trim() || pointWizardDraft.label,
    supportSurfaceZ: Number($("#pointWizardSurfaceZ").value || 0) / 1000,
    uses: $("#pointWizardUseDestination").checked ? ["waypoint", "destination"] : ["waypoint"],
  });
  if (!payload.ok) throw new Error(payload.error || "Point could not be saved.");
  applySceneSnapshot(payload);
  closePointWizard();
  if (payload.point) {
    showInspectorTab();
    setSelection("point", payload.point.id);
  }
  updateStatus(`Saved ${payload.point?.label || "taught point"}.`);
}

function renderPartInspector(container, part) {
  const title = document.createElement("h3");
  title.textContent = `Part: ${part.label}`;
  container.append(title);

  const shapes = [
    ["box", "Box"], ["open-box", "Box (open lid)"], ["rectangle", "Rectangle (flat)"],
    ["circle", "Circle (flat)"], ["cylinder", "Cylinder"], ["sphere", "Sphere"],
  ];

  const position = section("Position");
  const positionGrid = document.createElement("div");
  positionGrid.className = "field-grid three";
  positionGrid.append(
    numberField("X", part.position.x, 0.005, "position.x", (v) => { part.position.x = clamp(v, -SCENE_BOUND_METERS, SCENE_BOUND_METERS); savePart(part); }),
    numberField("Y", part.position.y, 0.005, "position.y", (v) => { part.position.y = clamp(v, -SCENE_BOUND_METERS, SCENE_BOUND_METERS); savePart(part); }),
    numberField("Z", part.position.z, 0.005, "position.z", (v) => { part.position.z = clamp(v, 0, 0.35); savePart(part); }),
  );
  position.body.append(positionGrid);
  container.append(position.details);

  const size = section("Size");
  const sizeGrid = document.createElement("div");
  sizeGrid.className = "field-grid three";
  sizeGrid.append(
    numberField("L", part.size.x, 0.005, "size.x", (v) => { part.size.x = clamp(v, 0.008, 0.2); savePart(part); }),
    numberField("W", part.size.y, 0.005, "size.y", (v) => { part.size.y = clamp(v, 0.008, 0.2); savePart(part); }),
    numberField("H", part.size.z, 0.005, "size.z", (v) => { part.size.z = clamp(v, 0.008, 0.2); savePart(part); }),
  );
  size.body.append(sizeGrid);
  container.append(size.details);

  const appearance = section("Appearance");
  appearance.body.append(
    textField("Name", part.label, (value) => { part.label = value.trim() || part.id; savePart(part); }),
    selectField("Shape", part.type, shapes, (value) => {
      part.type = value;
      // Flat shapes default to a thin height so they read as plates.
      if ((part.type === "rectangle" || part.type === "circle") && part.size.z > 0.02) {
      part.size.z = 0.01;
      part.position.z = 0.005;
      }
      savePart(part);
    }),
    numberField("Rotation", part.orientationDeg, 1, "orientationDeg", (v) => { part.orientationDeg = clamp(v, -180, 180); savePart(part); }),
    colorField("Color", part.color, "#2f80ed", (value) => { part.color = value; savePart(part); }),
    checkboxField("Graspable", part.graspable, (value) => { part.graspable = value; savePart(part); }),
  );
  container.append(appearance.details);

  const pickup = section("Pickup Setup", false);
  part.pickupProfiles ||= {};
  const activeTool = state.endEffector || "adaptive_gripper";
  const defaults = activeTool === "suction_gripper"
    ? { offsetLocalM: { x: 0, y: 0, z: 0 }, contactPreloadM: 0.002, yawMode: "minimum_joint_travel" }
    : { offsetLocalM: { x: 0, y: 0, z: 0 }, jawYawMode: "automatic_narrow_side", jawYawOverrideDeg: null, maximumTiltDeg: 10 };
  const profile = part.pickupProfiles[activeTool] ||= structuredClone(defaults);
  profile.offsetLocalM ||= { x: 0, y: 0, z: 0 };
  const offsetGrid = document.createElement("div");
  offsetGrid.className = "field-grid three";
  const offsetField = (label, axis) => numberField(
    `${label} (mm)`, Number(profile.offsetLocalM[axis] || 0) * 1000, 0.5,
    `pickup.${axis}`,
    (value) => {
      profile.offsetLocalM[axis] = clamp(value, axis === "z" ? -50 : -100, axis === "z" ? 50 : 100) / 1000;
      savePart(part);
    },
  );
  offsetGrid.append(offsetField("Local X", "x"), offsetField("Local Y", "y"), offsetField("Local Z", "z"));
  pickup.body.append(offsetGrid);
  const explanation = document.createElement("div");
  explanation.className = "helper-text";
  explanation.textContent = activeTool === "suction_gripper"
    ? "Defaults to the top-face center. X/Y rotate with the AprilTag part yaw; Z trims contact only."
    : "Defaults to a centered narrow-side pinch. X/Y rotate with the AprilTag part yaw.";
  pickup.body.append(explanation);
  if (activeTool === "adaptive_gripper") {
    pickup.body.append(selectField(
      "Jaw orientation", profile.jawYawOverrideDeg == null ? "auto" : "manual",
      [["auto", "Automatic narrow side"], ["manual", "Manual angle"]],
      (value) => {
        profile.jawYawOverrideDeg = value === "manual" ? Number(profile.jawYawOverrideDeg || 0) : null;
        profile.jawYawMode = value === "manual" ? "manual" : "automatic_narrow_side";
        savePart(part);
        renderInspector();
      },
    ));
    if (profile.jawYawOverrideDeg != null) {
      pickup.body.append(numberField("Jaw angle from part length", profile.jawYawOverrideDeg, 1, "pickup.jawYaw", (value) => {
        profile.jawYawOverrideDeg = clamp(value, -180, 180);
        savePart(part);
      }));
    }
  } else {
    pickup.body.append(numberField("Compliant preload (mm)", Number(profile.contactPreloadM || 0.002) * 1000, 0.5, "pickup.preload", (value) => {
      profile.contactPreloadM = clamp(value, 0, 8) / 1000;
      savePart(part);
    }));
  }
  const resetPickup = document.createElement("button");
  resetPickup.type = "button";
  resetPickup.textContent = "Reset Automatic Pickup";
  resetPickup.addEventListener("click", async () => {
    part.pickupProfiles[activeTool] = structuredClone(defaults);
    await savePart(part);
    renderInspector();
  });
  pickup.body.append(resetPickup);
  container.append(pickup.details);

  const advanced = section("Advanced", false);
  const meta = document.createElement("div");
  meta.className = "helper-text";
  const badges = [part.trackId ? "Tracked" : null, part.dimensionSource && part.dimensionSource !== "catalog" ? "Estimated size" : null, part.reservedByPlan ? "Reserved by plan" : null, state.calibration?.verification?.testingBypass ? "Unverified" : null].filter(Boolean);
  meta.textContent = `id: ${part.id}  -  source: ${part.source}${badges.length ? `  -  ${badges.join(" · ")}` : ""}`;
  advanced.body.append(meta);
  container.append(advanced.details);

  const tagButton = document.createElement("button");
  tagButton.type = "button";
  tagButton.textContent = part.trackingMode === "apriltag" ? "Change AprilTag" : "Attach AprilTag";
  tagButton.addEventListener("click", () => openPartWizard(part));
  container.append(tagButton);
  if (part.trackingMode === "apriltag") {
    const unbind = document.createElement("button");
    unbind.type = "button";
    unbind.textContent = "Make Virtual Only";
    unbind.addEventListener("click", async () => applySceneSnapshot(await post("/api/scene/part/tag-unbind", { partId: part.id })));
    container.append(unbind);
  }

  const del = document.createElement("button");
  del.type = "button";
  del.className = "danger";
  del.textContent = "Delete Part";
  del.addEventListener("click", async () => {
    applySceneSnapshot(await post("/api/scene/part/delete", { id: part.id }));
    setSelection("robot", null);
  });
  container.append(del);
}

function renderBinInspector(container, bin) {
  const title = document.createElement("h3");
  title.textContent = `Bin: ${bin.label}`;
  container.append(title);

  const position = section("Position");
  const positionGrid = document.createElement("div");
  positionGrid.className = "field-grid three";
  const markSimulated = () => {
    bin.positionStatus = "simulation_only";
    bin.positionSource = "dashboard_edit";
  };
  positionGrid.append(
    numberField("X", bin.position.x, 0.005, "position.x", (v) => { markSimulated(); bin.position.x = clamp(v, -SCENE_BOUND_METERS, SCENE_BOUND_METERS); saveBin(bin); }),
    numberField("Y", bin.position.y, 0.005, "position.y", (v) => { markSimulated(); bin.position.y = clamp(v, -SCENE_BOUND_METERS, SCENE_BOUND_METERS); saveBin(bin); }),
    numberField("Z", bin.position.z, 0.005, "position.z", (v) => { markSimulated(); bin.position.z = clamp(v, 0, 0.2); saveBin(bin); }),
  );
  position.body.append(positionGrid);
  const verification = document.createElement("div");
  verification.className = bin.positionStatus === "operator_verified" ? "success-callout" : "tag-pose-warning";
  verification.textContent = bin.positionStatus === "operator_verified"
    ? "Physical position confirmed."
    : "Simulation only: move the real bin to this location before physical use.";
  position.body.append(verification);
  if (bin.positionStatus !== "operator_verified") {
    const confirm = document.createElement("button");
    confirm.type = "button";
    confirm.textContent = "I Moved the Real Bin Here";
    confirm.addEventListener("click", async () => {
      applySceneSnapshot(await post("/api/scene/bin/confirm-position", { binId: bin.id }));
      renderInspector();
      updateStatus(`Confirmed ${bin.label}'s physical position.`);
    });
    position.body.append(confirm);
  }
  container.append(position.details);

  const size = section("Size");
  const sizeGrid = document.createElement("div");
  sizeGrid.className = "field-grid three";
  sizeGrid.append(
    numberField("L", bin.outer.x, 0.005, "outer.x", (v) => { bin.outer.x = clamp(v, 0.04, 0.4); saveBin(bin); }),
    numberField("W", bin.outer.y, 0.005, "outer.y", (v) => { bin.outer.y = clamp(v, 0.04, 0.4); saveBin(bin); }),
    numberField("H", bin.outer.z, 0.005, "outer.z", (v) => { bin.outer.z = clamp(v, 0.01, 0.2); saveBin(bin); }),
  );
  size.body.append(sizeGrid);
  container.append(size.details);

  const appearance = section("Appearance");
  appearance.body.append(
    textField("Name", bin.label, (value) => { bin.label = value.trim() || bin.id; saveBin(bin); }),
    numberField("Rotation", bin.orientationDeg, 1, "orientationDeg", (v) => { markSimulated(); bin.orientationDeg = clamp(v, -180, 180); saveBin(bin); }),
    colorField("Color", bin.color, "#f59e0b", (value) => { bin.color = value; saveBin(bin); }),
  );
  container.append(appearance.details);

  const advanced = section("Advanced", false);
  advanced.body.append(numberField("Inset", bin.wallThickness, 0.001, "wallThickness", (v) => { bin.wallThickness = clamp(v, 0.003, 0.03); saveBin(bin); }));
  if (bin.geometry) {
    const geo = document.createElement("div");
    geo.className = "helper-text";
    geo.textContent =
      `Drop boundary (auto): ${bin.geometry.interior.x.toFixed(3)} x ${bin.geometry.interior.y.toFixed(3)} m inside walls, ` +
      `floor z ${bin.geometry.floorZ.toFixed(3)}, wall top z ${bin.geometry.wallTopZ.toFixed(3)}`;
    advanced.body.append(geo);
  }
  const meta = document.createElement("div");
  meta.className = "helper-text";
  meta.textContent = `id: ${bin.id}`;
  advanced.body.append(meta);
  container.append(advanced.details);

  const del = document.createElement("button");
  del.type = "button";
  del.className = "danger";
  del.textContent = "Delete Bin";
  del.addEventListener("click", async () => {
    applySceneSnapshot(await post("/api/scene/bin/delete", { id: bin.id }));
    setSelection("robot", null);
  });
  container.append(del);
}

function renderPointInspector(container, point) {
  const title = document.createElement("h3");
  title.textContent = `Point: ${point.label}`;
  container.append(title);
  const tcp = point.tcpPoseM || {};
  const position = tcp.position || {};
  const rpy = tcp.rpyDeg || {};
  const summary = document.createElement("div");
  summary.className = "wizard-result";
  summary.textContent =
    `TCP ${(Number(position.x || 0) * 1000).toFixed(1)}, ${(Number(position.y || 0) * 1000).toFixed(1)}, ${(Number(position.z || 0) * 1000).toFixed(1)} mm · ` +
    `RPY ${Number(rpy.rx || 0).toFixed(1)}/${Number(rpy.ry || 0).toFixed(1)}/${Number(rpy.rz || 0).toFixed(1)}°`;
  container.append(summary);
  const details = section("Captured Robot State");
  const meta = document.createElement("div");
  meta.className = "helper-text";
  meta.textContent =
    `Tool: ${point.endEffector}. Joints: ${(point.jointAnglesDeg || []).map((value) => Number(value).toFixed(1)).join(", ")}°. ` +
    `Uses: ${(point.uses || []).join(" and ")}. Support surface: ${point.supportSurfaceZ == null ? "not set" : `${(Number(point.supportSurfaceZ) * 1000).toFixed(1)} mm`}.`;
  details.body.append(meta);
  container.append(details.details);
  const go = document.createElement("button");
  go.type = "button";
  go.className = "primary";
  go.textContent = "Plan & Simulate Move Here";
  go.addEventListener("click", async () => {
    state.activeProgramId = null;
    state.draftName = `Go to ${point.label}`;
    state.draftSteps = [{ type: "move_to_point", pointId: point.id }];
    renderProgramEditor();
    await planAndSimulate();
  });
  container.append(go);
  const del = document.createElement("button");
  del.type = "button";
  del.className = "danger";
  del.textContent = "Delete Point";
  del.addEventListener("click", async () => {
    applySceneSnapshot(await post("/api/scene/point/delete", { pointId: point.id }));
    setSelection("robot", null);
  });
  container.append(del);
}

export function renderInspector() {
  const objectPanel = $("#objectInspector");
  const robotPanel = $("#robotInspector");
  const sel = state.selection;
  if (sel.kind === "part" || sel.kind === "bin" || sel.kind === "point") {
    robotPanel.style.display = "none";
    objectPanel.style.display = "";
    objectPanel.innerHTML = "";
    const target = sel.kind === "part" ? findPart(sel.id) : sel.kind === "bin" ? findBin(sel.id) : findPoint(sel.id);
    if (!target) { objectPanel.innerHTML = '<div class="helper-text">Nothing selected.</div>'; return; }
    if (sel.kind === "part") renderPartInspector(objectPanel, target);
    else if (sel.kind === "bin") renderBinInspector(objectPanel, target);
    else renderPointInspector(objectPanel, target);
  } else {
    objectPanel.style.display = "none";
    robotPanel.style.display = "";
    renderToolOrientationStatus();
    renderPickCalibration();
    renderToolContactCalibration();
  }
}

// --------------------------------------------------------- robot controls

export function buildJointControls() {
  const jointInputs = $("#jointInputs");
  const angleReadout = $("#angleReadout");
  jointInputs.innerHTML = "";
  angleReadout.innerHTML = "";
  for (let index = 0; index < 6; index += 1) {
    const joint = index + 1;
    const [min, max] = state.limits[joint];
    const row = document.createElement("div");
    row.className = "joint-row";
    const label = document.createElement("span");
    label.textContent = `J${joint}`;
    const slider = document.createElement("input");
    slider.type = "range";
    slider.min = min; slider.max = max; slider.step = "0.1";
    slider.value = state.targetAngles[index];
    slider.dataset.joint = String(joint);
    const number = document.createElement("input");
    number.type = "number";
    number.min = min; number.max = max; number.step = "0.1";
    number.value = state.targetAngles[index].toFixed(1);
    number.dataset.joint = String(joint);
    slider.addEventListener("input", () => {
      state.targetAngles[index] = Number(slider.value);
      number.value = Number(slider.value).toFixed(1);
    });
    number.addEventListener("input", () => {
      const value = clamp(Number(number.value), min, max);
      state.targetAngles[index] = value;
      slider.value = String(value);
    });
    row.append(label, slider, number);
    jointInputs.append(row);

    const readout = document.createElement("div");
    readout.className = "readout-item";
    readout.innerHTML = `<strong>J${joint}</strong><span id="readout-${joint}">0.00</span>`;
    angleReadout.append(readout);
  }
}

export function setTargetInputs(angles) {
  state.targetAngles = angles.map(Number);
  const jointInputs = $("#jointInputs");
  for (let index = 0; index < 6; index += 1) {
    const joint = index + 1;
    const slider = jointInputs.querySelector(`input[type="range"][data-joint="${joint}"]`);
    const number = jointInputs.querySelector(`input[type="number"][data-joint="${joint}"]`);
    if (slider && number) {
      slider.value = String(state.targetAngles[index]);
      number.value = state.targetAngles[index].toFixed(1);
    }
  }
}

export function updateAngleReadouts() {
  for (let index = 0; index < 6; index += 1) {
    const el = document.querySelector(`#readout-${index + 1}`);
    if (el) el.textContent = `${state.angles[index].toFixed(2)}`;
    const programEl = document.querySelector(`#program-readout-${index + 1}`);
    if (programEl) programEl.textContent = `${state.angles[index].toFixed(2)}°`;
  }
}

// --------------------------------------------------------- program editor

function invalidatePlanPreview(message = "Program changed. Plan again before running.") {
  const plan = state.lastPlan;
  state.lastPlan = null;
  clearSimulation();
  if (plan?.objectSnapshots?.length) {
    post("/api/program/release-preview", { plan }).catch(() => {
      // Never block local editing just because preview cleanup was interrupted.
    });
  }
  if (message && plan) $("#planOutput").textContent = message;
}

function newStepId() {
  return globalThis.crypto?.randomUUID?.() || `step-${Date.now()}-${Math.random().toString(16).slice(2)}`;
}

function cloneValue(value) {
  return globalThis.structuredClone ? structuredClone(value) : JSON.parse(JSON.stringify(value));
}

function runPolicyFor(program = {}) {
  const policy = program.runPolicy || {};
  return {
    mode: policy.mode || "finite",
    cycleCount: Number(policy.cycleCount || program.repeatCount || 1),
    maxCycles: policy.maxCycles == null ? null : Number(policy.maxCycles),
    triggerPartId: policy.triggerPartId || null,
    stableFrames: Number(policy.stableFrames || 3),
    rearmAbsentMs: Number(policy.rearmAbsentMs || 1000),
    cooldownMs: Number(policy.cooldownMs ?? 500),
    xyEnvelopeM: Number(policy.xyEnvelopeM || 0.015),
    yawEnvelopeDeg: Number(policy.yawEnvelopeDeg || 10),
    expectedSurfaceId: policy.expectedSurfaceId || null,
  };
}

function escapeHtml(value) {
  return String(value ?? "").replace(/[&<>"']/g, (character) => ({
    "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;",
  })[character]);
}

function markProgramChanged(message) {
  state.programDirty = true;
  invalidatePlanPreview(message);
  renderProgramEditor();
}

function promptUnsavedProgram() {
  return new Promise((resolve) => {
    const dialog = document.createElement("dialog");
    dialog.className = "confirmation-dialog";
    dialog.innerHTML = `
      <form method="dialog" class="confirmation-card">
        <h2>Unsaved program changes</h2>
        <p>Save this program before closing the programmer?</p>
        <div class="program-inspector-actions">
          <button value="cancel">Cancel</button>
          <button value="discard">Discard</button>
          <button value="save" class="primary">Save</button>
        </div>
      </form>`;
    document.body.append(dialog);
    dialog.addEventListener("close", () => {
      const value = dialog.returnValue || "cancel";
      dialog.remove();
      resolve(value);
    }, { once: true });
    dialog.showModal();
  });
}

async function saveProgram() {
  state.draftRunPolicy.cycleCount = Number(state.draftRepeatCount || 1);
  const payload = await post("/api/programs/save", {
    id: state.activeProgramId || undefined,
    name: state.draftName,
    editorVersion: 2,
    repeatCount: state.draftRepeatCount,
    runPolicy: state.draftRunPolicy,
    steps: state.draftSteps,
  });
  if (!payload.ok && payload.error) throw new Error(payload.error);
  applySceneSnapshot(payload);
  if (payload.program) {
    state.activeProgramId = payload.program.id;
    state.draftSteps = payload.program.steps.map(cloneValue);
    state.draftRepeatCount = Number(payload.program.repeatCount || 1);
    state.draftRunPolicy = runPolicyFor(payload.program);
  }
  state.programDirty = false;
  renderTree();
  renderProgramEditor();
  updateStatus(`Saved program ${state.draftName}.`);
  return payload;
}

async function deleteProgram(programId) {
  const program = state.programs.find((item) => item.id === programId);
  if (!program) {
    const message = `Program '${programId}' no longer exists.`;
    $("#planOutput").textContent = message;
    updateStatus(message);
    return false;
  }
  if (!window.confirm(`Delete ${program.name}? This cannot be undone.`)) return false;
  const deletingActiveProgram = state.activeProgramId === programId;
  if (deletingActiveProgram) invalidatePlanPreview(null);
  try {
    const payload = await post("/api/programs/delete", { id: programId });
    if (payload.ok === false) throw new Error(payload.error || "Program could not be deleted.");
    applySceneSnapshot(payload);
    if (deletingActiveProgram) {
      state.activeProgramId = null;
      state.draftName = `Program ${state.programs.length + 1}`;
      state.draftRepeatCount = 1;
      state.draftRunPolicy = runPolicyFor({ repeatCount: 1 });
      state.draftSteps = [];
      state.selectedProgramStepId = null;
      state.programDirty = false;
    }
    renderTree();
    renderProgramEditor();
    updateStatus(`Deleted ${program.name}.`);
    return true;
  } catch (error) {
    const message = `Delete failed: ${error.message}`;
    $("#planOutput").textContent = message;
    updateStatus(message);
    return false;
  }
}

export async function closeProgramWorkspace({ force = false } = {}) {
  if (state.programDirty && !force) {
    const choice = await promptUnsavedProgram();
    if (choice === "cancel") return false;
    if (choice === "save") {
      try { await saveProgram(); }
      catch (error) {
        $("#planOutput").textContent = `Save failed: ${error.message}`;
        return false;
      }
    } else if (choice === "discard") {
      const saved = state.programs.find((program) => program.id === state.activeProgramId);
      if (saved) {
        state.draftName = saved.name;
        state.draftRepeatCount = Number(saved.repeatCount || 1);
        state.draftRunPolicy = runPolicyFor(saved);
        state.draftSteps = saved.steps.map(cloneValue);
      } else {
        state.draftSteps = [];
        state.draftRepeatCount = 1;
        state.draftRunPolicy = runPolicyFor({ repeatCount: 1 });
      }
      state.selectedProgramStepId = state.draftSteps[0]?.id || null;
      state.programDirty = false;
      invalidatePlanPreview(null);
    }
  }
  await stopJogging();
  const dialog = $("#programWorkspace");
  if (viewportHome && $("#robotViewport").parentElement !== viewportHome) viewportHome.append($("#robotViewport"));
  if (dialog.open) dialog.close();
  state.programWorkspaceOpen = false;
  state.editingWaypointStepId = null;
  window.dispatchEvent(new Event("resize"));
  return true;
}

export function openProgramWorkspace() {
  const dialog = $("#programWorkspace");
  if (!viewportHome) viewportHome = $("#robotViewport").parentElement;
  $("#programViewportHost").append($("#robotViewport"));
  if (!dialog.open) dialog.showModal();
  state.programWorkspaceOpen = true;
  renderProgramEditor();
  window.dispatchEvent(new Event("resize"));
}

async function loadProgram(programId) {
  const program = state.programs.find((p) => p.id === programId);
  if (!program) return;
  if (state.programDirty && state.activeProgramId !== programId) {
    const choice = await promptUnsavedProgram();
    if (choice === "cancel") return;
    if (choice === "save") {
      try { await saveProgram(); }
      catch (error) { $("#planOutput").textContent = `Save failed: ${error.message}`; return; }
    }
  }
  invalidatePlanPreview(null);
  state.activeProgramId = program.id;
  state.draftName = program.name;
  state.draftRepeatCount = Number(program.repeatCount || 1);
  state.draftRunPolicy = runPolicyFor(program);
  state.draftSteps = program.steps.map(cloneValue);
  const cachedPlan = ["cached", "validated"].includes(program.compiledCycle?.status)
    ? program.compiledCycle?.planTemplate
    : null;
  state.lastPlan = cachedPlan ? cloneValue(cachedPlan) : null;
  state.selectedProgramStepId = state.draftSteps[0]?.id || null;
  state.programDirty = false;
  renderTree();
  renderProgramEditor();
  if (state.lastPlan?.ok) renderPlanPath(state.lastPlan);
  openProgramWorkspace();
}

function stepLabel(step) {
  if (step.label) return step.label;
  if (step.type === "move" || step.type === "move_to_point") {
    const point = findPoint(step.pointId);
    const kind = (step.motionType || "joint") === "linear" ? "Linear Move" : "Joint Move";
    return `${kind}${point ? ` · ${point.label}` : step.waypoint?.label ? ` · ${step.waypoint.label}` : ""}`;
  }
  if (step.type === "pick") {
    const part = findPart(step.objectId);
    const registered = state.registeredParts.find((item) => item.partId === step.objectId);
    return `Pick ${part?.label || registered?.label || step.objectId}`;
  }
  if (step.type === "place") {
    if (step.binId) {
      const bin = findBin(step.binId);
      return `Place in ${bin ? bin.label : step.binId}`;
    }
    if (step.pointId) {
      const point = findPoint(step.pointId);
      return `Place at ${point ? point.label : step.pointId}`;
    }
    return `Place at point`;
  }
  if (step.type === "tool" || step.type === "acquire" || step.type === "release") {
    const action = step.action || step.type;
    if (state.endEffector === "suction_gripper") return action === "release" ? "Suction Off" : "Suction On";
    return action === "release" ? "Release Tool" : "Acquire Tool";
  }
  if (step.type === "wait") return `Wait ${(Number(step.durationMs || 1000) / 1000).toFixed(1)} s`;
  return "Home";
}

function stepProblem(step, index = 0) {
  if (step.enabled === false) return "";
  const prefix = index ? `Step ${index}: ` : "";
  if (step.type === "pick") {
    if (!step.objectId) return `${prefix}pick step has no part selected.`;
    if (!findPart(step.objectId)) {
      const registered = state.registeredParts.find((part) => part.partId === step.objectId);
      if (registered) return `${prefix}${registered.label || step.objectId} is registered but its AprilTag is not visible.`;
      return `${prefix}missing part '${step.objectId}'.`;
    }
  }
  if (step.type === "place") {
    if (!step.binId && !step.position && !step.pointId) return `${prefix}place step has no bin or point.`;
    if (step.binId && !findBin(step.binId)) return `${prefix}missing bin '${step.binId}'.`;
    if (step.pointId && !findPoint(step.pointId)) return `${prefix}missing taught point '${step.pointId}'.`;
  }
  if (step.type === "move" || step.type === "move_to_point") {
    if (step.pointId && !findPoint(step.pointId)) return `${prefix}missing taught point '${step.pointId}'.`;
    if (!step.pointId && !step.waypoint) return `${prefix}motion has not been taught.`;
    const point = step.pointId ? findPoint(step.pointId) : step.waypoint;
    if (point?.endEffector && point.endEffector !== state.endEffector) {
      return `${prefix}waypoint was captured for a different tool.`;
    }
  }
  return "";
}

function draftProgramProblem() {
  if (!state.draftSteps.length) return "Add steps to the program first.";
  if (state.draftRunPolicy.mode === "object_triggered" && !state.draftRunPolicy.triggerPartId) {
    return "Select the registered AprilTag part that starts each cycle.";
  }
  let holding = null;
  for (let index = 0; index < state.draftSteps.length; index += 1) {
    const step = state.draftSteps[index];
    if (step.enabled === false) continue;
    const problem = stepProblem(step, index + 1);
    if (problem) return problem;
    if (step.type === "pick") {
      if (holding) return `Step ${index + 1}: place '${holding}' before picking another part.`;
      holding = step.objectId;
    }
    if (step.type === "place") {
      if (!holding) return `Step ${index + 1}: place step needs a preceding pick.`;
      holding = null;
    }
  }
  if (holding) return `Program ends still holding '${holding}'; add a place step.`;
  return "";
}

function physicalRunBlocker() {
  const problem = draftProgramProblem();
  if (problem) return problem;
  const plan = state.lastPlan;
  if (!plan?.ok) return "Validate and simulate this program first.";
  if (plan.requiresCapturedToolRpy) return "Capture the tool orientation, then validate again.";
  if (plan.physicalReady === false) {
    const simulatedBin = (plan.unverifiedDestinations || []).find((item) => item.kind === "bin");
    if (simulatedBin) {
      return `Move the real ${simulatedBin.label || "bin"} to its simulated location and confirm it before running.`;
    }
    const coordinateError = plan.coordinatePreflight?.errors?.[0];
    return coordinateError?.message
      || coordinateError?.error
      || plan.coordinatePreview?.error
      || plan.error
      || "The validated preview is not ready for physical execution.";
  }
  if (plan.coordinatePreflight?.ok === false) {
    const error = plan.coordinatePreflight.errors?.[0];
    return error?.message || error?.error || "Coordinate preflight failed.";
  }
  if (plan.mode === "coordinate_program" && plan.coordinatePreview?.ok !== true) {
    return plan.coordinatePreview?.error || "Complete-path coordinate validation did not pass.";
  }
  return "";
}

function stepMeta(step) {
  if (step.type === "move" || step.type === "move_to_point") {
    const source = step.pointId ? (findPoint(step.pointId)?.label || "Missing shared point") : (step.waypoint ? "Embedded waypoint" : "Point required");
    return `${source} · speed ${Number(step.speed || 20)}`;
  }
  if (step.type === "pick") return findPart(step.objectId)?.label || "Part required";
  if (step.type === "place") return step.binId ? (findBin(step.binId)?.label || "Bin required") : step.pointId ? (findPoint(step.pointId)?.label || "Point required") : "Destination required";
  if (step.type === "wait") return `${Number(step.durationMs || 1000)} ms`;
  if (step.type === "tool") return state.endEffector === "suction_gripper" ? "Air suction gripper" : "Adaptive gripper";
  return "Validated home configuration";
}

function waypointSummary(point) {
  if (!point) return '<div class="waypoint-summary"><strong>No waypoint captured</strong><span>Use Edit Point to jog and teach this motion.</span></div>';
  const angles = (point.jointAnglesDeg || []).map((value) => Number(value).toFixed(1)).join(", ");
  const position = point.tcpPoseM?.position || {};
  return `<div class="waypoint-summary"><strong>${escapeHtml(point.label || "Embedded waypoint")}</strong><code>J [${angles || "—"}]</code><code>TCP ${Number(position.x || 0).toFixed(3)}, ${Number(position.y || 0).toFixed(3)}, ${Number(position.z || 0).toFixed(3)} m</code><span>${escapeHtml(point.endEffector || state.endEffector)}</span></div>`;
}

function makeSelect(options, value, onChange) {
  const select = document.createElement("select");
  for (const [id, label] of options) {
    const option = document.createElement("option");
    option.value = id;
    option.textContent = label;
    option.selected = id === value;
    select.append(option);
  }
  select.addEventListener("change", () => onChange(select.value));
  return select;
}

function renderWaypointEditor(step) {
  const panel = $("#programStepInspector");
  const jointMode = state.jogTab === "joint";
  const tcpMode = state.jogTab === "tcp";
  panel.innerHTML = `
    <div><span class="eyebrow">Teach Waypoint</span><h2>Edit Point</h2></div>
    <div class="jog-tabs"><button data-jog-tab="joint" class="${jointMode ? "active" : ""}">Joint Jog</button><button data-jog-tab="tcp" class="${tcpMode ? "active" : ""}">TCP Jog</button><button data-jog-tab="guide" class="${state.jogTab === "guide" ? "active" : ""}">Hand Guide</button></div>
    <div id="jogEditorBody"></div>
    <div id="jogStatus" class="helper-text">Jogging invalidates the current path. Hold controls stop on release.</div>
    <div class="program-inspector-actions"><button id="cancelEditPointBtn">Back</button><button id="saveEmbeddedPointBtn" class="primary">Save Point</button></div>`;
  const body = $("#jogEditorBody");
  if (jointMode) {
    body.innerHTML = `
      <div class="jog-settings">
        <label>Mode<select id="jogMode"><option value="hold">Hold</option><option value="increment">Increment</option></select></label>
        <label>Speed<input id="jogSpeed" type="number" min="1" max="30" value="${state.jogSpeed}"></label>
        <label>Increment<select id="jointJogIncrement"><option value="0.5">0.5°</option><option value="1">1°</option><option value="5">5°</option></select></label>
      </div>
      <div class="jog-grid">${state.angles.map((angle, index) => `<div class="jog-row"><strong>J${index + 1}</strong><span class="jog-value" id="program-readout-${index + 1}">${Number(angle).toFixed(2)}°</span><button data-jog-joint="${index + 1}" data-jog-dir="-1"${state.endEffector === "suction_gripper" && index === 5 ? " disabled title=\"J6 is locked for suction\"" : ""}>−</button><button data-jog-joint="${index + 1}" data-jog-dir="1"${state.endEffector === "suction_gripper" && index === 5 ? " disabled title=\"J6 is locked for suction\"" : ""}>+</button></div>`).join("")}</div>
      ${state.endEffector === "suction_gripper" ? '<div class="jog-warning">J6 is locked because the suction cup is rotationally symmetric.</div>' : ""}`;
    $("#jogMode").value = state.jogMode || "hold";
    $("#jointJogIncrement").value = String(state.jogIncrementJoint);
  } else if (tcpMode) {
    const axes = [["X", 1], ["Y", 2], ["Z", 3], ["Rx", 4], ["Ry", 5], ["Rz", 6]];
    body.innerHTML = `
      <div class="jog-warning">Robot-base frame: +X front, +Y left, +Z up. TCP jogging is incremental and validated before each command.</div>
      <div class="jog-settings"><label>Translation<select id="tcpJogIncrement"><option value="1">1 mm</option><option value="5">5 mm</option><option value="10">10 mm</option></select></label><label>Rotation<select id="rotationJogIncrement"><option value="0.5">0.5°</option><option value="1">1°</option><option value="5">5°</option></select></label><label>Speed<input id="jogSpeed" type="number" min="1" max="30" value="${state.jogSpeed}"></label></div>
      <div class="jog-grid">${axes.map(([label, axis]) => `<div class="jog-row"><strong>${label}</strong><span class="jog-value">${axis <= 3 ? "mm" : "deg"}</span><button data-jog-axis="${axis}" data-jog-dir="-1">−</button><button data-jog-axis="${axis}" data-jog-dir="1">+</button></div>`).join("")}</div>`;
    $("#tcpJogIncrement").value = String(state.jogIncrementTcp);
    $("#rotationJogIncrement").value = String(state.jogIncrementRotation);
  } else {
    body.innerHTML = `<div class="waypoint-summary"><strong>Hand guiding</strong><span>Release the joints, guide the installed tool to the pose, then save while the robot is stationary.</span></div><button id="programReleaseRobotBtn" type="button">Release Robot Joints</button>`;
  }
  panel.querySelectorAll("[data-jog-tab]").forEach((button) => button.addEventListener("click", async () => {
    await stopJogging();
    state.jogTab = button.dataset.jogTab;
    renderWaypointEditor(step);
  }));
  $("#cancelEditPointBtn").addEventListener("click", async () => {
    await stopJogging();
    state.editingWaypointStepId = null;
    renderProgramEditor();
  });
  $("#saveEmbeddedPointBtn").addEventListener("click", () => captureWaypoint(step));
  $("#programReleaseRobotBtn")?.addEventListener("click", async () => {
    try {
      const result = await post("/api/command/release", {});
      $("#jogStatus").textContent = result.ok ? "Joints released. Move the robot, let it settle, then Save Point." : result.error;
    } catch (error) { $("#jogStatus").textContent = error.message; }
  });
}

function renderSelectedStepInspector() {
  const panel = $("#programStepInspector");
  const step = state.draftSteps.find((item) => item.id === state.selectedProgramStepId);
  if (!step) {
    panel.innerHTML = '<div class="empty-program-inspector"><h2>No command selected</h2><p>Select a node in the program tree or add a command.</p></div>';
    return;
  }
  if (state.editingWaypointStepId === step.id) {
    renderWaypointEditor(step);
    return;
  }
  panel.innerHTML = `<div><span class="eyebrow">Command settings</span><h2>${stepLabel(step)}</h2></div>`;
  const enabled = document.createElement("label");
  enabled.className = "wizard-check";
  enabled.innerHTML = '<input type="checkbox"> Command enabled';
  enabled.querySelector("input").checked = step.enabled !== false;
  enabled.querySelector("input").addEventListener("change", (event) => { step.enabled = event.target.checked; markProgramChanged(); });
  panel.append(enabled);
  const label = document.createElement("input");
  label.value = step.label || "";
  label.placeholder = "Optional command name";
  label.addEventListener("change", () => { step.label = label.value.trim() || null; markProgramChanged(); });
  panel.append(fieldRow("Name", label));

  if (step.type === "move" || step.type === "move_to_point") {
    const motion = makeSelect([["joint", "Joint Move"], ["linear", "Linear Move"]], step.motionType || "joint", (value) => { step.type = "move"; step.motionType = value; markProgramChanged(); });
    panel.append(fieldRow("Motion type", motion));
    const speed = document.createElement("input");
    speed.type = "number"; speed.min = "1"; speed.max = "100"; speed.value = String(step.speed || 20);
    speed.addEventListener("change", () => { step.speed = clamp(speed.value, 1, 100); markProgramChanged(); });
    panel.append(fieldRow("Speed", speed));
    const source = makeSelect([["embedded", "Embedded waypoint"], ["shared", "Shared point"]], step.pointId ? "shared" : "embedded", (value) => {
      if (value === "shared") {
        step.pointId = state.taughtPoints[0]?.id || null;
        delete step.waypoint;
      } else {
        const linked = findPoint(step.pointId);
        if (linked) step.waypoint = { ...cloneValue(linked), id: `${step.id}-waypoint`, label: `${stepLabel(step)} Point` };
        delete step.pointId;
      }
      markProgramChanged();
    });
    panel.append(fieldRow("Point source", source));
    if (step.pointId) {
      panel.append(fieldRow("Shared point", makeSelect(state.taughtPoints.map((point) => [point.id, point.label]), step.pointId, (value) => { step.pointId = value; markProgramChanged(); })));
    }
    const point = step.pointId ? findPoint(step.pointId) : step.waypoint;
    const summary = document.createElement("div");
    summary.innerHTML = waypointSummary(point);
    panel.append(summary);
    const waypointActions = document.createElement("div");
    waypointActions.className = "program-inspector-actions";
    waypointActions.innerHTML = `<button data-step-action="edit-point">Edit Point</button><button data-step-action="copy-point"${point ? "" : " disabled"}>Save Copy</button>${step.pointId ? '<button data-step-action="detach-point">Detach Copy</button>' : ""}`;
    panel.append(waypointActions);
  } else if (step.type === "pick") {
    panel.append(fieldRow("Part", makeSelect(state.parts.map((part) => [part.id, part.label]), step.objectId, (value) => { step.objectId = value; markProgramChanged(); })));
  } else if (step.type === "place") {
    const destinationType = step.pointId ? "point" : "bin";
    panel.append(fieldRow("Destination type", makeSelect([["bin", "Bin"], ["point", "Taught point"]], destinationType, (value) => {
      delete step.binId; delete step.pointId; delete step.position;
      if (value === "point") step.pointId = state.taughtPoints[0]?.id || null;
      else step.binId = state.bins[0]?.id || null;
      markProgramChanged();
    })));
    if (destinationType === "point") panel.append(fieldRow("Point", makeSelect(state.taughtPoints.map((point) => [point.id, point.label]), step.pointId, (value) => { step.pointId = value; markProgramChanged(); })));
    else panel.append(fieldRow("Bin", makeSelect(state.bins.map((bin) => [bin.id, bin.label]), step.binId, (value) => { step.binId = value; markProgramChanged(); })));
  } else if (step.type === "tool") {
    panel.append(fieldRow("Action", makeSelect([["acquire", state.endEffector === "suction_gripper" ? "Suction On" : "Acquire"], ["release", state.endEffector === "suction_gripper" ? "Suction Off" : "Release"]], step.action, (value) => { step.action = value; markProgramChanged(); })));
  } else if (step.type === "wait") {
    const duration = document.createElement("input");
    duration.type = "number"; duration.min = "0.05"; duration.max = "600"; duration.step = "0.1"; duration.value = String(Number(step.durationMs || 1000) / 1000);
    duration.addEventListener("change", () => { step.durationMs = Math.round(clamp(duration.value, .05, 600) * 1000); markProgramChanged(); });
    panel.append(fieldRow("Seconds", duration));
  } else {
    const info = document.createElement("div");
    info.className = "waypoint-summary";
    info.innerHTML = "<strong>Home</strong><span>Moves to the validated home joint configuration.</span>";
    panel.append(info);
  }
  const actions = document.createElement("div");
  actions.className = "program-inspector-actions";
  actions.innerHTML = '<button data-step-action="up">Move Up</button><button data-step-action="down">Move Down</button><button data-step-action="duplicate">Duplicate</button><button data-step-action="delete" class="danger">Delete</button>';
  panel.append(actions);
}

export function renderProgramEditor() {
  const nameInput = $("#progName");
  if (!nameInput) return;
  if (document.activeElement !== nameInput) nameInput.value = state.draftName;
  $("#programRepeatCount").value = String(state.draftRepeatCount || 1);
  $("#programRunMode").value = state.draftRunPolicy.mode || "finite";
  $("#programMaxCycles").value = state.draftRunPolicy.maxCycles == null ? "" : String(state.draftRunPolicy.maxCycles);
  const triggerSelect = $("#programTriggerPart");
  triggerSelect.innerHTML = '<option value="">Select registered part</option>';
  for (const definition of state.registeredParts) {
    const option = document.createElement("option");
    option.value = definition.partId;
    option.textContent = `${definition.label} · tag ${definition.tagId}`;
    option.selected = definition.partId === state.draftRunPolicy.triggerPartId;
    triggerSelect.append(option);
  }
  $("#programCycleCountField").hidden = state.draftRunPolicy.mode !== "finite";
  $("#programMaxCyclesField").hidden = state.draftRunPolicy.mode === "finite";
  $("#programTriggerPartField").hidden = state.draftRunPolicy.mode !== "object_triggered";
  $("#programEnvelopeField").hidden = state.draftRunPolicy.mode !== "object_triggered";
  $("#programXyEnvelope").value = String(Math.round(Number(state.draftRunPolicy.xyEnvelopeM || 0.015) * 1000));
  $("#programYawEnvelope").value = String(Number(state.draftRunPolicy.yawEnvelopeDeg || 10));
  $("#triggerCycleBtn").hidden = state.draftRunPolicy.mode !== "external_triggered";
  $("#programDirtyBadge").textContent = state.programDirty ? "Unsaved" : "Saved";
  $("#programDirtyBadge").classList.toggle("dirty", state.programDirty);
  $("#programConnectionState").textContent = state.executing ? "Running" : state.connected ? "Online" : "Offline";
  $("#programConnectionState").classList.toggle("online", state.connected && !state.executing);
  $("#programConnectionState").classList.toggle("running", Boolean(state.executing));
  $("#programToolState").textContent = state.endEffector === "suction_gripper" ? "Air Suction Gripper" : "Adaptive Gripper";

  const list = $("#stepList");
  list.innerHTML = "";
  const insert = (index) => {
    const button = document.createElement("button");
    button.type = "button"; button.className = "program-insert-point"; button.title = "Insert command here";
    button.addEventListener("click", () => {
      state.programInsertIndex = index;
      $("#commandPalette").hidden = false;
    });
    list.append(button);
  };
  insert(0);
  state.draftSteps.forEach((step, index) => {
    if (!step.id) step.id = newStepId();
    const problem = stepProblem(step, index + 1);
    const node = document.createElement("button");
    node.type = "button";
    node.className = `program-node${step.id === state.selectedProgramStepId ? " selected" : ""}${problem ? " invalid" : ""}${step.enabled === false ? " disabled" : ""}${step.id === state.simulationSourceStepId || step.id === state.executionSourceStepId ? " active" : ""}`;
    node.innerHTML = `<span class="program-node-number">${index + 1}</span><span class="program-node-text"><span class="program-node-title"></span><span class="program-node-meta"></span></span><span class="node-status ${problem ? "warning" : ""}">${problem ? "!" : "✓"}</span>`;
    node.querySelector(".program-node-title").textContent = stepLabel(step);
    node.querySelector(".program-node-meta").textContent = problem || stepMeta(step);
    node.addEventListener("click", () => {
      state.selectedProgramStepId = step.id;
      state.editingWaypointStepId = null;
      renderProgramEditor();
    });
    list.append(node);
    insert(index + 1);
  });
  if (!state.draftSteps.length) {
    const empty = document.createElement("div");
    empty.className = "empty-program-inspector";
    empty.innerHTML = "<strong>No commands yet</strong><p>Select Add Command to begin.</p>";
    list.append(empty);
  }
  renderSelectedStepInspector();
  const blocker = physicalRunBlocker();
  const runButton = $("#runBtn");
  const runtimeState = state.productionRuntime?.state || "disarmed";
  const productionActive = !["disarmed", "completed", "faulted"].includes(runtimeState);
  const running = Boolean(state.physicalRunActive || state.executing || productionActive);
  runButton.disabled = running;
  if (running) {
    runButton.textContent = productionActive ? "Armed" : "Running";
    $("#runStatus").textContent = productionActive
      ? `${runtimeState.replaceAll("_", " ")} · cycle ${Number(state.productionRuntime.cycleCount || 0)}${state.productionRuntime.maxCycles ? ` / ${state.productionRuntime.maxCycles}` : ""}`
      : state.executionSourceStepId
        ? `Running ${stepLabel(state.draftSteps.find((step) => step.id === state.executionSourceStepId) || {})}.`
        : "Physical program is running.";
  } else if (blocker) {
    runButton.textContent = state.lastPlan?.ok ? "Blocked — View Issue" : "Validate & Simulate First";
    $("#runStatus").textContent = blocker;
  } else {
    runButton.textContent = state.draftRunPolicy.mode === "finite" ? "Run Complete Program" : "Arm Program";
    const active = state.programs.find((program) => program.id === state.activeProgramId);
    const cache = active?.compiledCycle;
    $("#runStatus").textContent = cache?.status === "cached"
      ? "Cached cycle is validated and ready."
      : "Validated complete program is ready.";
  }
}

function insertProgramCommand(kind) {
  const base = { id: newStepId(), enabled: true, label: null };
  let step;
  if (kind === "joint" || kind === "linear") {
    step = { ...base, type: "move", motionType: kind, speed: 20 };
  } else if (kind === "pick") {
    step = { ...base, type: "pick", objectId: state.parts[0]?.id || null };
  } else if (kind === "place") {
    step = state.bins.length
      ? { ...base, type: "place", binId: state.bins[0].id }
      : { ...base, type: "place", pointId: state.taughtPoints[0]?.id || null };
  } else if (kind === "acquire" || kind === "release") {
    step = { ...base, type: "tool", action: kind };
  } else if (kind === "wait") {
    step = { ...base, type: "wait", durationMs: 1000 };
  } else {
    step = { ...base, type: "home" };
  }
  const index = clamp(state.programInsertIndex, 0, state.draftSteps.length);
  state.draftSteps.splice(index, 0, step);
  state.selectedProgramStepId = step.id;
  $("#commandPalette").hidden = true;
  markProgramChanged();
}

async function stopJogging() {
  clearInterval(jogHeartbeatTimer);
  clearInterval(tcpJogTimer);
  jogHeartbeatTimer = null;
  tcpJogTimer = null;
  activeJogSessionId = null;
  try { await post("/api/robot/jog/stop", {}); } catch { /* watchdog also stops motion */ }
}

async function jointJog(button) {
  const jointId = Number(button.dataset.jogJoint);
  const sign = Number(button.dataset.jogDir);
  state.jogSpeed = clamp($("#jogSpeed")?.value || 10, 1, 30);
  state.jogIncrementJoint = Number($("#jointJogIncrement")?.value || 1);
  state.jogMode = $("#jogMode")?.value || "hold";
  invalidatePlanPreview(null);
  if (state.jogMode === "increment") {
    const result = await post("/api/robot/jog/step", {
      space: "joint", axisId: jointId,
      increment: sign * state.jogIncrementJoint,
      speed: state.jogSpeed,
    });
    const status = $("#jogStatus");
    if (status) status.textContent = result.ok ? `J${jointId} stepped ${sign * state.jogIncrementJoint}°.` : result.error;
    return;
  }
  const result = await post("/api/robot/jog/start", {
    jointId,
    direction: sign > 0 ? 1 : 0,
    speed: state.jogSpeed,
  });
  if (!result.ok) throw new Error(result.error || "Jog could not start.");
  activeJogSessionId = result.jog?.sessionId;
  jogHeartbeatTimer = setInterval(async () => {
    if (!activeJogSessionId) return;
    try {
      const heartbeat = await post("/api/robot/jog/heartbeat", { sessionId: activeJogSessionId });
      if (!heartbeat.ok) await stopJogging();
    } catch { await stopJogging(); }
  }, 250);
}

async function tcpJog(button) {
  const axisId = Number(button.dataset.jogAxis);
  const sign = Number(button.dataset.jogDir);
  state.jogSpeed = clamp($("#jogSpeed")?.value || 10, 1, 30);
  state.jogIncrementTcp = Number($("#tcpJogIncrement")?.value || 5);
  state.jogIncrementRotation = Number($("#rotationJogIncrement")?.value || 1);
  const increment = sign * (axisId <= 3 ? state.jogIncrementTcp : state.jogIncrementRotation);
  invalidatePlanPreview(null);
  const result = await post("/api/robot/jog/step", {
    space: "tcp", axisId, increment, speed: state.jogSpeed,
  });
  const status = $("#jogStatus");
  if (status) status.textContent = result.ok ? `TCP ${axisId} stepped ${increment}${axisId <= 3 ? " mm" : "°"}.` : result.error;
}

async function captureWaypoint(step) {
  const status = $("#jogStatus");
  try {
    await stopJogging();
    if (status) status.textContent = "Waiting for the robot to settle and capturing both joint and TCP data…";
    const payload = await post("/api/robot/points/capture", {
      label: step.waypoint?.label || `${stepLabel(step)} Point`,
      persist: false,
    });
    if (!payload.ok || !payload.pointDraft) throw new Error(payload.error || "The pose could not be captured.");
    const point = { ...payload.pointDraft, id: `${step.id}-waypoint` };
    if (step.pointId) {
      point.id = step.pointId;
      const saved = await post("/api/scene/point", point);
      if (!saved.ok && saved.error) throw new Error(saved.error);
      applySceneSnapshot(saved);
    } else {
      step.waypoint = point;
    }
    state.editingWaypointStepId = null;
    state.programDirty = true;
    invalidatePlanPreview(null);
    renderProgramEditor();
    updateStatus(`Captured ${stepLabel(step)}.`);
  } catch (error) {
    if (status) status.textContent = `Capture failed: ${error.message}`;
  }
}

async function handleStepAction(action) {
  const index = state.draftSteps.findIndex((item) => item.id === state.selectedProgramStepId);
  if (index < 0) return;
  const step = state.draftSteps[index];
  if (action === "edit-point") {
    state.editingWaypointStepId = step.id;
    renderProgramEditor();
    return;
  }
  if (action === "copy-point") {
    const point = step.pointId ? findPoint(step.pointId) : step.waypoint;
    if (!point) return;
    try {
      const copy = cloneValue(point);
      delete copy.id;
      copy.label = `${point.label || stepLabel(step)} Copy`;
      const result = await post("/api/scene/point", copy);
      if (!result.ok && result.error) throw new Error(result.error);
      applySceneSnapshot(result);
      updateStatus(`Saved ${copy.label} to Points.`);
    } catch (error) { $("#runStatus").textContent = `Point copy failed: ${error.message}`; }
    return;
  }
  if (action === "detach-point") {
    const point = findPoint(step.pointId);
    if (point) step.waypoint = { ...cloneValue(point), id: `${step.id}-waypoint`, label: `${point.label} Copy` };
    delete step.pointId;
  } else if (action === "duplicate") {
    const copy = cloneValue(step);
    copy.id = newStepId();
    if (copy.waypoint) copy.waypoint.id = `${copy.id}-waypoint`;
    copy.label = copy.label ? `${copy.label} Copy` : null;
    state.draftSteps.splice(index + 1, 0, copy);
    state.selectedProgramStepId = copy.id;
  } else if (action === "delete") {
    state.draftSteps.splice(index, 1);
    state.selectedProgramStepId = state.draftSteps[Math.min(index, state.draftSteps.length - 1)]?.id || null;
  } else if (action === "up" && index > 0) {
    [state.draftSteps[index - 1], state.draftSteps[index]] = [state.draftSteps[index], state.draftSteps[index - 1]];
  } else if (action === "down" && index < state.draftSteps.length - 1) {
    [state.draftSteps[index + 1], state.draftSteps[index]] = [state.draftSteps[index], state.draftSteps[index + 1]];
  } else return;
  markProgramChanged();
}

function selectSimulationCommand(offset) {
  if (!state.draftSteps.length || !state.lastPlan?.ok) return;
  let index = state.draftSteps.findIndex((step) => step.id === state.selectedProgramStepId);
  index = clamp(index + offset, 0, state.draftSteps.length - 1);
  const step = state.draftSteps[index];
  state.selectedProgramStepId = step.id;
  if (!state.simulation) startSimulation(state.lastPlan);
  pauseSimulation();
  seekSimulationSource(step.id, 0);
  state.simulationSourceStepId = step.id;
  renderProgramEditor();
}

function planSummary(plan) {
  const timing = plan?.planningDiagnostics || {};
  const timingLine = Number.isFinite(Number(timing.totalMs))
    ? `Planning: ${(Number(timing.totalMs) / 1000).toFixed(2)}s; slowest phase ${String(timing.slowestPhase || "unknown").replace(/Ms$/, "")}.`
    : null;
  if (!plan?.ok) return [plan?.error || "Plan failed.", timingLine].filter(Boolean).join("\n");
  const model = plan.motionModel || {};
  const rpy = model.toolRpySource === "canonical_top_down"
    ? ", per-pick top-down RPY"
    : model.toolRpyDeg
    ? `, RPY ${Number(model.toolRpyDeg.rx || 0).toFixed(1)}/${Number(model.toolRpyDeg.ry || 0).toFixed(1)}/${Number(model.toolRpyDeg.rz || 0).toFixed(1)}`
    : ", capture required";
  const preview = plan.coordinatePreview?.ok
    ? `, preview ${Number(plan.coordinatePreview.solvedStates || 0)} states`
    : "";
  const lines = [
    `Plan: ${plan.program}  -  ${plan.steps.length} states, est ${(plan.durationMs / 1000).toFixed(1)}s`,
    `Motion: ${model.type || plan.mode || "coordinate"} (${model.toolRpySource || "runtime_current"}${rpy}${preview})`,
  ];
  if (timingLine) lines.push(timingLine);
  if (plan.requiresCapturedToolRpy) {
    lines.push("NOT READY: capture tool orientation before physical execution.");
  }
  if (plan.coordinatePreflight && !plan.coordinatePreflight.ok) {
    const first = plan.coordinatePreflight.errors?.[0];
    lines.push(`NOT READY: ${first?.message || "coordinate preflight failed."}`);
  }
  if (plan.mode === "coordinate_program" && !plan.coordinatePreview?.ok) {
    lines.push(`Preview: path only${plan.coordinatePreview?.error ? ` (${plan.coordinatePreview.error})` : ""}.`);
    const failed = (plan.coordinatePreview?.states || []).find((item) => item?.ok === false);
    if (failed) {
      const reasons = (failed.rejectionReasons || []).join(", ").replaceAll("_", " ");
      const residual = Number(failed.jawCenterErrorMm ?? failed.positionErrorMm);
      lines.push(
        `First rejected point: ${failed.stateId || "unknown"}`
        + `${Number.isFinite(residual) ? `, predicted offset ${residual.toFixed(1)} mm` : ""}`
        + `${reasons ? ` (${reasons})` : ""}.`
      );
    }
    if (plan.coordinatePreview?.correctiveGuidance) {
      lines.push(`Reach guidance: ${plan.coordinatePreview.correctiveGuidance}`);
    }
  }
  if (plan.coordinatePreview?.ok && Array.isArray(plan.coordinatePreview.states)) {
    const maxJawError = Math.max(...plan.coordinatePreview.states.map((item) => Number(item.jawCenterErrorMm || 0)));
    const maxPlannedError = Math.max(...plan.coordinatePreview.states.map((item) => Number(item.plannedJawCenterErrorMm || 0)));
    const maxTilt = Math.max(...plan.coordinatePreview.states.map((item) => Number(item.toolApproachTiltDeg || 0)));
    lines.push(
      `Validated jaw center: IK ${maxJawError.toFixed(2)} mm, flange/TCP ${maxPlannedError.toFixed(2)} mm, `
      + `vertical-axis error ${maxTilt.toFixed(2)} deg.`
    );
  }
  const pickDescend = (plan.steps || []).find((step) => step.name === "descend" && step.grasp?.heightModel);
  if (pickDescend) {
    const height = pickDescend.grasp.heightModel;
    lines.push(
      `Pick depth: object Z ${Number(height.objectBottomZ).toFixed(3)}–${Number(height.objectTopZ).toFixed(3)} m, `
      + `jaw ${Number(height.jawCenterTargetZ).toFixed(3)} m, fingertip low ${Number(height.fingertipLowTargetZ).toFixed(3)} m, `
      + `overlap ${Number(height.actualFingerOverlapM * 1000).toFixed(1)} mm, table clearance ${Number(height.tableClearanceM * 1000).toFixed(1)} mm.`
    );
  }
  for (const note of plan.notes || []) lines.push(`note: ${note}`);
  lines.push(plan.safetyGate.reason);
  return lines.join("\n");
}

async function planAndSimulate() {
  invalidatePlanPreview(null);
  const problem = draftProgramProblem();
  if (problem) {
    state.lastPlan = null;
    $("#planOutput").textContent = problem;
    renderProgramEditor();
    return;
  }
  const planningStarted = performance.now();
  $("#planOutput").textContent = "Planning… 0.0s";
  const planningTimer = window.setInterval(() => {
    const elapsed = (performance.now() - planningStarted) / 1000;
    $("#planOutput").textContent = `Planning… ${elapsed.toFixed(1)}s`;
  }, 100);
  try {
    // A validated production cycle belongs to a persisted program. Save the
    // operator's current definition first, then let the server atomically
    // attach the compiled cycle only after full validation succeeds.
    if (state.programDirty || !state.activeProgramId) await saveProgram();
    const plan = await post("/api/program/plan", {
      programId: state.activeProgramId,
      persistCompiledCycle: true,
    });
    state.lastPlan = plan;
    $("#planOutput").textContent = planSummary(plan);
    if (plan.ok) {
      try { applySceneSnapshot(await api("/api/scene")); } catch { /* cache badge refresh is best-effort */ }
      renderPlanPath(plan);
      const canSimulateCoordinate = plan.mode !== "coordinate_program" || Boolean(plan.coordinatePreview?.ok);
      if (!plan.requiresCapturedToolRpy && canSimulateCoordinate && plan.coordinatePreflight?.ok !== false) {
        startSimulation(plan);
      } else {
        clearSimulation({ preservePath: true });
        renderPlanPath(plan);
        if (plan.requiresCapturedToolRpy) {
          updateStatus("Plan path shown. Capture tool orientation, then plan again to simulate/run.");
        } else if (plan.coordinatePreflight?.ok === false) {
          updateStatus("Plan path shown. Coordinate preflight must pass before simulate/run.");
        } else {
          updateStatus("Plan path shown. Firmware preview angles are unavailable, so simulation is disabled.");
        }
      }
    }
  } catch (error) {
    state.lastPlan = null;
    $("#planOutput").textContent = `Plan failed: ${error.message}`;
  } finally {
    window.clearInterval(planningTimer);
  }
  renderProgramEditor();
}

async function runPhysical() {
  if (state.physicalRunActive || state.executing) {
    updateStatus("A physical program is already running.");
    return;
  }
  const blocker = physicalRunBlocker();
  if (blocker) {
    const message = `Physical run blocked: ${blocker}`;
    $("#planOutput").textContent = `${state.lastPlan ? planSummary(state.lastPlan) : "No validated preview."}\n\n${message}`;
    updateStatus(message);
    renderProgramEditor();
    return;
  }
  const plan = state.lastPlan;
  const unverifiedCamera = Boolean(state.calibration?.verification?.testingBypass && !state.calibration?.verification?.passed);
  const ok = window.confirm(
    unverifiedCamera
      ? "Run this program on the PHYSICAL robot using camera coordinates that have not passed the optional nine-point accuracy check?\n\nKeep the workspace clear and be ready to press Stop. Fresh AprilTag and stale-preview checks still apply."
      : "Run this program on the PHYSICAL robot?\nSpeeds are limited; Stop aborts at any waypoint."
  );
  if (!ok) return;
  if (state.activeProgramId) {
    try {
      clearSimulation();
      const result = await post("/api/program/runtime/arm", {
        programId: state.activeProgramId,
        confirm: "RUN_PHYSICAL_PICK",
        speedOverridePct: clamp($("#programSpeedOverride")?.value || 100, 1, 100),
      });
      if (!result.ok) throw new Error(result.error || "Program could not be armed.");
      state.productionRuntime = result;
      updateStatus(result.mode === "finite" ? "Program running." : "Production program armed.");
      renderProgramEditor();
      return;
    } catch (error) {
      $("#planOutput").textContent = `${planSummary(plan)}\n\nArm failed: ${error.message}`;
      updateStatus(`Arm failed: ${error.message}`);
      renderProgramEditor();
      return;
    }
  }
  $("#runBtn").disabled = true;
  state.physicalRunActive = true;
  state.executionSourceStepId = null;
  clearSimulation();
  updateStatus("Running program on robot...");
  $("#planOutput").textContent = "Physical run started...";
  try {
    const speedOverridePct = clamp($("#programSpeedOverride")?.value || 100, 1, 100);
    const result = await post("/api/program/execute", {
      plan,
      confirm: "RUN_PHYSICAL_PICK",
      speedOverridePct,
    });
    state.connected = result.ok || state.connected;
    state.lastError = result.ok ? null : result.error;
    const lines = (result.executedSteps || []).map((s) => {
      const error = Number.isFinite(Number(s.motion?.errorDeg)) ? `, err ${Number(s.motion.errorDeg).toFixed(2)} deg` : "";
      const coordError = Number.isFinite(Number(s.motion?.maxPositionErrorMm)) ? `, xyz err ${Number(s.motion.maxPositionErrorMm).toFixed(1)} mm` : "";
      const misses = Number(s.motion?.feedbackMisses || 0) > 0 ? `, feedback misses ${Number(s.motion.feedbackMisses)}` : "";
      const target = s.motion?.targetAngles ? `, target [${s.motion.targetAngles.join(", ")}]` : "";
      const coords = s.motion?.targetCoords ? `, coords [${s.motion.targetCoords.join(", ")}]` : "";
      const actualCoords = s.motion?.actualCoords ? `, actual [${s.motion.actualCoords.join(", ")}]` : "";
      const reason = s.motion?.failureReason ? `, reason ${s.motion.failureReason}` : "";
      const controller = s.motion?.controllerError
        ? `, controller ${s.motion.controllerError.code ?? "?"} (${String(s.motion.controllerError.label || "unknown").replaceAll("_", " ")})`
        : "";
      const hostReach = s.motion?.ikValidation?.hostIkReachable === false
        ? ", host IK unreachable"
        : "";
      const source = s.motion?.startPoseSource ? `, start ${s.motion.startPoseSource}` : "";
      const rpySource = s.motion?.toolRpySource ? `, rpy ${s.motion.toolRpySource}` : "";
      const motion = s.motion
        ? `${s.motion.completion || s.motion.command}${error}${coordError}${misses}${target}${coords}${actualCoords}${reason}${controller}${hostReach}${source}${rpySource}`
        : "-";
      const gripResult = s.gripper?.gripperResult ? `/${s.gripper.gripperResult}` : "";
      const gripDiscarded = Number(s.gripper?.discardedFrames || 0) > 0 ? `, discarded ${Number(s.gripper.discardedFrames)}` : "";
      const grip = s.gripper?.feedback ? `, gripper ${s.gripper.feedback}${gripResult}${gripDiscarded}` : "";
      const verify = s.verification
        ? `, verify ${s.verification.verified === true ? "ok" : s.verification.verified === false ? "failed" : "unknown"}:${s.verification.verificationReason}`
        : "";
      const recovery = s.feedbackRecovery
        ? `, recovery reads ${Number(s.feedbackRecovery.recoveryReads || 0)}, recovery misses ${Number(s.feedbackRecovery.feedbackMisses || 0)}`
        : "";
      return `${s.stateId}: ${motion}${grip}${verify}${recovery}`;
    });
    if (result.cameraAccuracyWarning) lines.unshift(`WARNING: ${result.cameraAccuracyWarning}`);
    if (result.aborted) lines.push("STOPPED BY OPERATOR");
    else if (!result.ok) lines.push(`FAILED: ${result.error}${result.failedState ? ` at ${result.failedState}` : ""}`);
    const lastActualAngles = [...(result.executedSteps || [])].reverse()
      .map((s) => s.motion?.actualAngles)
      .find((angles) => Array.isArray(angles) && angles.length === 6);
    if (lastActualAngles) {
      state.angles = lastActualAngles.map(Number);
      state.renderAngles = [...state.angles];
      state.renderInitialized = true;
      updateAngleReadouts();
    }
    $("#planOutput").textContent = `${planSummary(plan)}\n\nPhysical run:\n${lines.join("\n") || "(no steps)"}`;
    updateStatus(result.aborted ? "Program stopped." : result.ok ? "Program finished." : "Program failed.");
    try { applySceneSnapshot(await api("/api/scene")); } catch { /* scene refresh best-effort */ }
  } catch (error) {
    state.lastError = error.message;
    const payload = error.payload || {};
    const failedState = payload.failedState ? ` at ${payload.failedState}` : "";
    const message = `Execution request failed${failedState}: ${error.message}`;
    $("#planOutput").textContent = `${planSummary(plan)}\n\nPhysical run:\nFAILED: ${message}`;
    updateStatus(message);
  } finally {
    // A physical attempt consumes the preview, even when preflight rejects it.
    // Replanning guarantees current camera poses and current robot angles.
    state.lastPlan = null;
    state.physicalRunActive = false;
    state.executionSourceStepId = null;
    renderProgramEditor();
  }
}

// ------------------------------------------------------------- wiring

export function initUI() {
  $("#treeRobot").addEventListener("click", () => {
    showInspectorTab();
    setSelection("robot", null);
  });

  $("#addPartBtn").addEventListener("click", () => openPartWizard());
  $("#partWizardCloseBtn").addEventListener("click", closePartWizard);
  $("#partWizardCancelBtn").addEventListener("click", closePartWizard);
  $("#partRelockCameraBtn").addEventListener("click", async () => {
    const button = $("#partRelockCameraBtn");
    button.disabled = true;
    button.textContent = "Relocking…";
    try {
      const payload = await post("/api/camera/calibration/accept-pose", {});
      if (!payload.ok) throw new Error(payload.error || "The current camera position could not be accepted.");
      if (payload.calibration) state.calibration = payload.calibration;
      $("#partTagSelectionStatus").textContent = "Camera relocked. Robot coordinates will resume on the next valid frame.";
      await refreshPartTagPicker();
    } catch (error) {
      $("#partTagSelectionStatus").textContent = `Relock failed: ${error.message}`;
    } finally {
      button.disabled = false;
      button.textContent = "Relock Camera Here";
    }
  });
  $("#partTrackingMode").addEventListener("change", async () => {
    const tagged = $("#partTrackingMode").value === "apriltag";
    $("#partTagPickerSection").hidden = !tagged;
    $("#partTagPlacement").hidden = !tagged;
    if (tagged) {
      try { await post("/api/camera/start", {}); } catch { /* picker reports failure */ }
      clearInterval(partWizardTagTimer);
      partWizardTagTimer = setInterval(refreshPartTagPicker, 150);
      refreshPartTagPicker();
    } else {
      clearInterval(partWizardTagTimer);
      $("#partTagCamera").removeAttribute("src");
    }
  });
  $("#partWizardUnits").addEventListener("change", () => {
    const next = $("#partWizardUnits").value;
    const oldFactor = measurementFactor(partWizardUnits);
    const newFactor = measurementFactor(next);
    for (const id of ["partWizardLength", "partWizardWidth", "partWizardHeight", "partTagOffsetX", "partTagOffsetY"]) {
      const input = $(`#${id}`);
      input.value = (Number(input.value || 0) * oldFactor / newFactor).toFixed(next === "mm" ? 1 : 2);
    }
    partWizardUnits = next;
  });
  $("#partTagYaw").addEventListener("change", () => {
    $("#partTagYawCustom").hidden = $("#partTagYaw").value !== "custom";
  });
  $("#partWizardForm").addEventListener("submit", async (event) => {
    event.preventDefault();
    try { await savePartWizard(); }
    catch (error) { $("#partWizardResult").textContent = error.message; }
  });

  $("#addBinBtn").addEventListener("click", async () => {
    try {
      const index = state.bins.length + 1;
      const payload = await post("/api/scene/bin", {
        label: `Bin ${String.fromCharCode(64 + index)}`,
        position: { x: 0.18, y: -0.10 - 0.02 * index, z: 0 },
        outer: { x: 0.14, y: 0.14, z: 0.02 },
      });
      applySceneSnapshot(payload);
      if (payload.bin) {
        showInspectorTab();
        setSelection("bin", payload.bin.id);
      }
      updateStatus(`Added ${payload.bin?.label || "bin"}.`);
    } catch (error) {
      updateStatus(`Add bin failed: ${error.message} - if you updated the code, restart web_server.py.`);
    }
  });

  $("#addSurfaceBtn").addEventListener("click", () => openSupportSurfaceDialog());
  $("#cancelSupportSurfaceBtn").addEventListener("click", () => $("#supportSurfaceDialog").close());
  $("#supportSurfaceForm").addEventListener("submit", async (event) => {
    try { await saveSupportSurface(event); }
    catch (error) { event.preventDefault(); updateStatus(`Surface save failed: ${error.message}`); }
  });
  $("#deleteSupportSurfaceBtn").addEventListener("click", async () => {
    const id = $("#supportSurfaceId").value;
    if (!id || !window.confirm("Delete this support surface? Cached production cycles will require validation again.")) return;
    try {
      const payload = await post("/api/scene/support-surface/delete", { id });
      if (!payload.ok) throw new Error(payload.error || "Surface could not be deleted.");
      applySceneSnapshot(payload);
      $("#supportSurfaceDialog").close();
      renderTree();
    } catch (error) { updateStatus(`Surface delete failed: ${error.message}`); }
  });

  $("#addPointBtn").addEventListener("click", openPointWizard);
  $("#pointWizardCloseBtn").addEventListener("click", closePointWizard);
  $("#pointWizardCancelBtn").addEventListener("click", closePointWizard);
  $("#pointWizardReleaseBtn").addEventListener("click", async () => {
    const button = $("#pointWizardReleaseBtn");
    button.disabled = true;
    try {
      const payload = await post("/api/command/release", {});
      if (!payload.ok) throw new Error(payload.error || "The robot joints could not be released.");
      $("#pointWizardReadout").textContent = "Joints released. Hand-guide the robot to the point, keep it still, then capture.";
    } catch (error) {
      $("#pointWizardReadout").textContent = `Release failed: ${error.message}`;
    } finally {
      button.disabled = false;
    }
  });
  $("#pointWizardCaptureBtn").addEventListener("click", async () => {
    const button = $("#pointWizardCaptureBtn");
    button.disabled = true;
    try { await capturePointWizardPose(); }
    catch (error) { $("#pointWizardReadout").textContent = `Capture failed: ${error.message}`; }
    finally { button.disabled = false; }
  });
  $("#pointWizardForm").addEventListener("submit", async (event) => {
    event.preventDefault();
    try { await savePointWizard(); }
    catch (error) { $("#pointWizardReadout").textContent = `Save failed: ${error.message}`; }
  });

  $("#addProgramBtn").addEventListener("click", async () => {
    if (state.programDirty) {
      const choice = await promptUnsavedProgram();
      if (choice === "cancel") return;
      if (choice === "save") {
        try { await saveProgram(); }
        catch (error) { $("#planOutput").textContent = `Save failed: ${error.message}`; return; }
      }
    }
    invalidatePlanPreview(null);
    state.activeProgramId = null;
    state.draftName = `Program ${state.programs.length + 1}`;
    state.draftRepeatCount = 1;
    state.draftRunPolicy = runPolicyFor({ repeatCount: 1 });
    state.draftSteps = [];
    state.selectedProgramStepId = null;
    state.programDirty = false;
    state.lastPlan = null;
    renderTree();
    renderProgramEditor();
    openProgramWorkspace();
  });

  $("#openProgrammerBtn").addEventListener("click", openProgramWorkspace);
  $("#closeProgrammerBtn").addEventListener("click", () => closeProgramWorkspace());
  $("#programWorkspace").addEventListener("cancel", (event) => {
    event.preventDefault();
    closeProgramWorkspace();
  });
  $("#progName").addEventListener("input", () => {
    invalidatePlanPreview();
    state.draftName = $("#progName").value.trim() || "Program";
    state.programDirty = true;
    renderProgramEditor();
  });
  $("#programRepeatCount").addEventListener("change", () => {
    state.draftRepeatCount = clamp($("#programRepeatCount").value, 1, 20);
    state.draftRunPolicy.cycleCount = state.draftRepeatCount;
    markProgramChanged();
  });
  $("#programRunMode").addEventListener("change", () => {
    state.draftRunPolicy.mode = $("#programRunMode").value;
    markProgramChanged();
  });
  $("#programMaxCycles").addEventListener("change", () => {
    const value = $("#programMaxCycles").value;
    state.draftRunPolicy.maxCycles = value ? clamp(value, 1, 1000000) : null;
    markProgramChanged();
  });
  $("#programTriggerPart").addEventListener("change", () => {
    const partId = $("#programTriggerPart").value || null;
    state.draftRunPolicy.triggerPartId = partId;
    const visible = state.parts.find((part) => part.id === partId);
    state.draftRunPolicy.expectedSurfaceId = visible?.supportSurfaceId || null;
    markProgramChanged();
  });
  $("#programXyEnvelope").addEventListener("change", () => {
    state.draftRunPolicy.xyEnvelopeM = clamp($("#programXyEnvelope").value, 1, 50) / 1000;
    markProgramChanged();
  });
  $("#programYawEnvelope").addEventListener("change", () => {
    state.draftRunPolicy.yawEnvelopeDeg = clamp($("#programYawEnvelope").value, 1, 45);
    markProgramChanged();
  });
  $("#commandPaletteBtn").addEventListener("click", () => {
    state.programInsertIndex = state.draftSteps.length;
    $("#commandPalette").hidden = !$("#commandPalette").hidden;
  });
  $("#commandPalette").addEventListener("click", (event) => {
    const button = event.target.closest("[data-command]");
    if (button) insertProgramCommand(button.dataset.command);
  });
  $("#programStepInspector").addEventListener("click", (event) => {
    const button = event.target.closest("[data-step-action]");
    if (button) handleStepAction(button.dataset.stepAction);
  });
  $("#programStepInspector").addEventListener("pointerdown", async (event) => {
    const joint = event.target.closest("[data-jog-joint]");
    const axis = event.target.closest("[data-jog-axis]");
    if (!joint && !axis) return;
    event.preventDefault();
    event.target.setPointerCapture?.(event.pointerId);
    try {
      if (joint) await jointJog(joint);
      else {
        await tcpJog(axis);
        tcpJogTimer = setInterval(() => tcpJog(axis).catch(() => stopJogging()), 280);
      }
    } catch (error) {
      const status = $("#jogStatus");
      if (status) status.textContent = `Jog failed: ${error.message}`;
      await stopJogging();
    }
  });
  for (const eventName of ["pointerup", "pointercancel", "pointerleave"]) {
    $("#programStepInspector").addEventListener(eventName, () => stopJogging());
  }
  window.addEventListener("blur", () => stopJogging());
  $("#saveProgramBtn").addEventListener("click", async () => {
    try { await saveProgram(); }
    catch (error) { $("#planOutput").textContent = `Save failed: ${error.message}`; }
  });
  $("#deleteProgramBtn").addEventListener("click", async () => {
    if (!state.activeProgramId) return;
    await deleteProgram(state.activeProgramId);
  });
  $("#planBtn").addEventListener("click", planAndSimulate);
  $("#runBtn").addEventListener("click", runPhysical);
  $("#triggerCycleBtn").addEventListener("click", async () => {
    try {
      const result = await post("/api/program/runtime/trigger", {});
      if (!result.ok) throw new Error(result.error || "Trigger was rejected.");
      state.productionRuntime = result;
      renderProgramEditor();
    } catch (error) { updateStatus(`Trigger failed: ${error.message}`); }
  });
  $("#programStopBtn").addEventListener("click", async () => {
    await stopJogging();
    try {
      state.productionRuntime = await post("/api/program/runtime/stop", {});
    } catch { /* ordinary Stop below remains authoritative */ }
    try { await post("/api/command/stop", {}); } catch { /* status poll reports link errors */ }
    updateStatus("Stop sent.");
  });
  setInterval(async () => {
    if (!state.programWorkspaceOpen) return;
    try {
      state.productionRuntime = await api("/api/program/runtime/status");
      renderProgramEditor();
    } catch { /* main status polling reports server loss */ }
  }, 400);
  $("#simPlayBtn").addEventListener("click", () => {
    if (!state.lastPlan?.ok) return;
    if (!state.simulation || state.simulation.done) startSimulation(state.lastPlan);
    else resumeSimulation();
  });
  $("#simPauseBtn").addEventListener("click", pauseSimulation);
  $("#simResetBtn").addEventListener("click", () => {
    if (state.lastPlan?.ok) { startSimulation(state.lastPlan); pauseSimulation(); }
  });
  $("#simPreviousBtn").addEventListener("click", () => selectSimulationCommand(-1));
  $("#simNextBtn").addEventListener("click", () => selectSimulationCommand(1));
  $("#simulateFromBtn").addEventListener("click", () => selectSimulationCommand(0));
  $("#programSpeedOverride").addEventListener("input", () => {
    $("#programSpeedOutput").textContent = `${$("#programSpeedOverride").value}%`;
  });

  // toolbar
  $("#stopBtn").addEventListener("click", async () => {
    try {
      await post("/api/command/stop", {});
      updateStatus("Stop sent.");
    } catch (error) {
      updateStatus(`Stop failed: ${error.message}`);
    }
  });

  // robot panel
  $("#refreshPortsBtn").addEventListener("click", loadPorts);
  $("#connectBtn").addEventListener("click", async () => {
    state.targetsInitialized = false;
    await post("/api/config", { port: $("#portSelect").value, baud: Number($("#baudInput").value) });
    updateStatus(`Using ${$("#portSelect").value}`);
  });
  $("#endEffectorSelect").addEventListener("change", async () => {
    const endEffector = $("#endEffectorSelect").value;
    try {
      applySceneSnapshot(await post("/api/scene/end-effector", { endEffector }));
      renderGripperActionLabels();
      renderPickCalibration();
      renderToolContactCalibration();
      await syncEndEffector(true);
      updateStatus(`Using ${$("#endEffectorSelect").selectedOptions[0]?.textContent || "end effector"}.`);
    } catch (error) {
      updateStatus(`End effector update failed: ${error.message}`);
      renderEndEffectorSelect();
    }
  });
  $("#captureToolOrientationBtn").addEventListener("click", async () => {
    try {
      const payload = await post("/api/robot/capture-tool-orientation", {});
      applySceneSnapshot(payload);
      renderToolOrientationStatus();
      renderPickCalibration();
      const rpy = payload.capturedToolRpyDeg;
      updateStatus(rpy
        ? `Captured tool orientation ${Number(rpy.rx).toFixed(1)}, ${Number(rpy.ry).toFixed(1)}, ${Number(rpy.rz).toFixed(1)}.`
        : "Captured tool orientation.");
    } catch (error) {
      updateStatus(`Capture failed: ${error.message}`);
    }
  });
  $("#clearToolOrientationBtn").addEventListener("click", async () => {
    try {
      applySceneSnapshot(await post("/api/scene/coordinate-planner", { toolRpyDeg: null }));
      renderToolOrientationStatus();
      renderPickCalibration();
      updateStatus("Coordinate planner will use runtime orientation.");
    } catch (error) {
      updateStatus(`Orientation update failed: ${error.message}`);
    }
  });
  $("#savePickCalibrationBtn").addEventListener("click", async () => {
    try {
      const payload = await post("/api/scene/coordinate-planner", {
        pickHeightBiasM: clamp(Number($("#pickHeightBiasInput").value), -8, 8) / 1000,
        minimumTableClearanceM: clamp(Number($("#minimumTableClearanceInput").value), 2, 12) / 1000,
      });
      applySceneSnapshot(payload);
      renderPickCalibration();
      updateStatus("Saved coordinate pick calibration.");
    } catch (error) {
      updateStatus(`Pick calibration failed: ${error.message}`);
    }
  });
  $("#resetPickCalibrationBtn").addEventListener("click", async () => {
    try {
      const payload = await post("/api/scene/coordinate-planner", {
        pickHeightBiasM: 0,
        minimumTableClearanceM: 0.004,
      });
      applySceneSnapshot(payload);
      renderPickCalibration();
      updateStatus("Reset coordinate pick calibration.");
    } catch (error) {
      updateStatus(`Pick calibration reset failed: ${error.message}`);
    }
  });
  $("#applyToolContactCalibrationBtn").addEventListener("click", async () => {
    try {
      const payload = await post("/api/scene/coordinate-planner", {
        calibrationJawYawDeg: latestPickJawYawDeg(),
        observedContactMissMm: {
          left: clamp(Number($("#toolMissLeftInput").value || 0), -30, 30),
          forward: clamp(Number($("#toolMissForwardInput").value || 0), -30, 30),
          high: clamp(Number($("#toolMissHighInput").value || 0), -30, 30),
        },
      });
      applySceneSnapshot(payload);
      ["#toolMissLeftInput", "#toolMissForwardInput", "#toolMissHighInput"].forEach((id) => { $(id).value = "0"; });
      renderToolContactCalibration();
      updateStatus("Applied the observed miss to this tool only.");
    } catch (error) { updateStatus(`Tool calibration failed: ${error.message}`); }
  });
  $("#resetToolContactCalibrationBtn").addEventListener("click", async () => {
    try {
      applySceneSnapshot(await post("/api/scene/coordinate-planner", {
        toolTcpCorrectionLocalM: { x: 0, y: 0, z: 0 },
      }));
      renderToolContactCalibration();
      updateStatus("Reset this tool's contact correction.");
    } catch (error) { updateStatus(`Tool calibration reset failed: ${error.message}`); }
  });
  for (const [id, command] of [
    ["#powerOnBtn", "power-on"], ["#focusBtn", "focus-all"], ["#releaseBtn", "release"],
    ["#homePoseBtn", "home"],
    ["#gripperOpenBtn", "gripper-open"], ["#gripperAutoBtn", "gripper-auto"], ["#gripperCloseBtn", "gripper-close"],
  ]) {
    $(id).addEventListener("click", async () => {
      if (command === "gripper-open") state.gripperTargetOpen = 0.08;
      if (command === "gripper-close" || command === "gripper-auto") state.gripperTargetOpen = 1;
      try {
        const payload = await post(`/api/command/${command}`, {});
        state.lastError = payload.ok ? null : payload.error;
        const action = payload.suction
          ? `Suction ${payload.suction.enabled ? "on" : "off"}`
          : command;
        if (payload.ok && command === "home") {
          setTargetInputs(HOME_ANGLES);
          state.targetsInitialized = true;
        }
        updateStatus(payload.ok ? `Sent ${action}.` : "Command failed.");
      } catch (error) {
        updateStatus(`Command failed: ${error.message}`);
      }
    });
  }
  for (const [id, command] of [
    ["#suctionPumpOnBtn", "suction-pump-on"],
    ["#suctionPumpOffBtn", "suction-pump-off"],
    ["#suctionValveOpenBtn", "suction-valve-open"],
    ["#suctionValveCloseBtn", "suction-valve-close"],
    ["#suctionFullOnBtn", "suction-on"],
    ["#suctionFullOffBtn", "suction-off"],
  ]) {
    $(id).addEventListener("click", async () => {
      try {
        const payload = await post(`/api/command/${command}`, {});
        state.lastError = payload.ok ? null : payload.error;
        const seq = (payload.suction?.sequence || [])
          .map((item) => item.type === "sleep" ? `sleep ${item.seconds}s` : `pin ${item.pin}=${item.signal}`)
          .join(", ");
        $("#suctionDiagStatus").textContent = payload.ok
          ? `${command}: outputs sent (${seq || "command complete"}). The robot cannot electrically confirm pump vibration or vacuum.`
          : `Diagnostic failed: ${payload.error}`;
        updateStatus(payload.ok ? `Sent ${command}.` : "Suction diagnostic failed.");
      } catch (error) {
        $("#suctionDiagStatus").textContent = `Diagnostic failed: ${error.message}`;
        updateStatus(`Suction diagnostic failed: ${error.message}`);
      }
    });
  }
  $("#cameraRefreshBtn").addEventListener("click", async () => {
    setCameraFormDirty(false);
    await refreshCameraDevices({ force: true }).catch(() => {});
    await refreshCameraStatus();
  });
  $("#cameraStartBtn").addEventListener("click", async () => {
    try {
      await saveCameraConfig();
      const payload = await post("/api/camera/start", {});
      state.cameraStatus = payload;
      renderCameraStatus();
      updateStatus(payload.running ? "Camera started." : "Camera did not start.");
    } catch (error) {
      updateStatus(`Camera start failed: ${error.message}`);
    }
  });
  $("#cameraStopBtn").addEventListener("click", async () => {
    try {
      state.cameraStatus = await post("/api/camera/stop", {});
      renderCameraStatus();
      updateStatus("Camera stopped.");
    } catch (error) {
      updateStatus(`Camera stop failed: ${error.message}`);
    }
  });
  $("#openCameraCalibrationBtn").addEventListener("click", async () => {
    calibrationWizardStep = 1;
    setCameraFormDirty(false);
    renderCameraConfig({ force: true });
    markerMapSaved = false;
    intrinsicsSolved = Boolean(state.calibration?.intrinsics?.ok) && Number(state.calibration?.intrinsics?.intrinsicRmsPx) <= 2.5 && Number(state.calibration?.intrinsics?.maximumViewErrorPx || state.calibration?.intrinsics?.intrinsicRmsPx || Infinity) <= 4;
    cameraPoseAccepted = Boolean(state.calibration?.fiducials?.baselineHomography);
    // A saved pose baseline can only be produced from a valid workspace-tag
    // frame, so restore both completed checks when reopening the wizard.
    workspaceVerified = cameraPoseAccepted;
    calibrationTestingBypass = Boolean(state.calibration?.verification?.testingBypass);
    accuracyVerified = Boolean(state.calibration?.verification?.passed || calibrationTestingBypass);
    renderCalibrationWizard();
    $("#cameraCalibrationDialog").showModal();
    if (!state.cameraStatus?.running) {
      try {
        await saveCameraConfig();
        state.cameraStatus = await post("/api/camera/start", {});
        renderCameraStatus();
      } catch (error) { $("#charucoStatus").textContent = `Camera could not start: ${error.message}`; }
    }
  });
  $("#closeCameraCalibrationBtn").addEventListener("click", () => $("#cameraCalibrationDialog").close());
  $("#calibrationBackBtn").addEventListener("click", () => { calibrationWizardStep = Math.max(1, calibrationWizardStep - 1); renderCalibrationWizard(); });
  $("#calibrationNextBtn").addEventListener("click", () => {
    if (!calibrationStepComplete()) {
      $("#calibrationStepHint").textContent = calibrationWizardStep === 3
        ? `Take ${Math.max(0, 12 - calibrationCaptureCount)} more accepted photo${12 - calibrationCaptureCount === 1 ? "" : "s"} before continuing.`
        : "Finish the required action on this step before continuing.";
      return;
    }
    calibrationWizardStep = Math.min(6, calibrationWizardStep + 1);
    renderCalibrationWizard();
  });
  $("#charucoClearBtn").addEventListener("click", async () => {
    const payload = await post("/api/camera/calibration/charuco/clear", {});
    updateCaptureProgress(payload.sampleCount, payload.diversity);
    $("#charucoStatus").textContent = "Photos cleared. Start again with the board centered.";
  });
  $("#charucoRemoveLastBtn").addEventListener("click", async () => {
    const payload = await post("/api/camera/calibration/charuco/remove-last", {});
    updateCaptureProgress(payload.sampleCount, payload.diversity);
    $("#charucoStatus").textContent = payload.sampleCount ? `Removed the last photo. ${payload.sampleCount} remain.` : "No calibration photos remain.";
  });
  $("#charucoCaptureBtn").addEventListener("click", async () => {
    try {
      const payload = await post("/api/camera/calibration/charuco/capture", {});
      updateCaptureProgress(payload.sampleCount, payload.diversity);
      $("#charucoStatus").textContent = payload.ok
        ? `Photo ${payload.sampleCount} accepted with ${payload.cornerCount} detected corners.`
        : `Capture rejected: ${payload.error}`;
    } catch (error) { $("#charucoStatus").textContent = `Capture failed: ${error.message}`; }
  });
  $("#charucoSolveBtn").addEventListener("click", async () => {
    try {
      const payload = await post("/api/camera/calibration/charuco/solve", {});
      if (payload.calibration) state.calibration = payload.calibration;
      intrinsicsSolved = Boolean(payload.ok);
      $("#charucoStatus").textContent = payload.ok
        ? `Intrinsics passed using ${payload.sampleCount} photos: RMS ${Number(payload.intrinsicRmsPx).toFixed(3)} px, worst photo ${Number(payload.maximumViewErrorPx).toFixed(3)} px.`
        : `Solve rejected: ${payload.error}${payload.diversity?.missing?.length ? ` Missing: ${payload.diversity.missing.join(", ").replaceAll("_", " ")}.` : ""}`;
    } catch (error) { $("#charucoStatus").textContent = `Solve failed: ${error.message}`; }
  });
  $("#fiducialSaveBtn").addEventListener("click", async () => {
    try {
      const current = state.calibration?.fiducials || {};
      const payload = await post("/api/camera/calibration/workspace", {
        fiducials: { ...current, ...fiducialsFromInputs(), baselineHomography: current.baselineHomography || null },
      });
      state.calibration = payload.calibration;
      markerMapSaved = Boolean(payload.ok);
      await saveCameraConfig();
      $("#markerMapStatus").textContent = "Measurements saved. Keep the tags fixed from now on.";
    } catch (error) { $("#fiducialStatus").textContent = `Marker save failed: ${error.message}`; }
  });
  $("#fiducialVerifyBtn").addEventListener("click", async () => {
    try { const payload = await post("/api/camera/calibration/verify", {}); workspaceVerified = Boolean(payload.ok); renderFiducialResult(payload); }
    catch (error) { $("#fiducialStatus").textContent = `Verification failed: ${error.message}`; }
  });
  $("#fiducialAcceptPoseBtn").addEventListener("click", async () => {
    const button = $("#fiducialAcceptPoseBtn");
    button.disabled = true;
    button.textContent = "Locking…";
    try {
      const payload = await post("/api/camera/calibration/accept-pose", {});
      if (payload.calibration) state.calibration = payload.calibration;
      cameraPoseAccepted = Boolean(payload.ok);
      // The accept endpoint recomputes and validates the tag homography, so a
      // successful lock also proves the workspace verification is current.
      if (payload.ok) {
        workspaceVerified = true;
        intrinsicsSolved = true;
      }
      $("#fiducialStatus").textContent = payload.ok ? "Camera pose baseline accepted." : `Pose rejected: ${payload.error}`;
      $("#fiducialDebugPreview").src = `/api/camera/debug-frame?t=${Date.now()}`;
      if (payload.ok && calibrationWizardStep === 4) {
        calibrationWizardStep = 5;
        renderCalibrationWizard();
      }
    } catch (error) { $("#fiducialStatus").textContent = `Pose acceptance failed: ${error.message}`; }
    finally {
      button.disabled = false;
      button.textContent = "Lock Camera Position";
    }
  });
  $("#verificationReportBtn").addEventListener("click", async () => {
    try {
      const samples = Array.from({ length: 9 }, (_, index) => {
        const expectedX = $(`[data-accuracy-expected-x="${index}"]`).value;
        const expectedY = $(`[data-accuracy-expected-y="${index}"]`).value;
        const measuredX = $(`[data-accuracy-measured-x="${index}"]`).dataset.value;
        const measuredY = $(`[data-accuracy-measured-y="${index}"]`).dataset.value;
        if ([expectedX, expectedY, measuredX, measuredY].some((value) => value === "" || value === undefined)) throw new Error(`Point ${index + 1} is incomplete.`);
        return { expected: { x: Number(expectedX), y: Number(expectedY) }, measured: { x: Number(measuredX), y: Number(measuredY) } };
      });
      $("#verificationReportStatus").textContent = "Hold the target still while five stability frames are checked...";
      const stationarySpreadM = await measureStationarySpread();
      const payload = await post("/api/camera/calibration/verification-report", { samples, stationarySpreadM });
      const report = payload.report || {};
      if (payload.calibration) state.calibration = payload.calibration;
      accuracyVerified = Boolean(report.passed);
      calibrationTestingBypass = false;
      $("#verificationReportStatus").textContent = `${report.passed ? "PASS" : "FAIL"}: ${report.sampleCount || 0} points, RMS ${(Number(report.rmsXyErrorM || 0) * 1000).toFixed(2)} mm, max ${(Number(report.maxXyErrorM || 0) * 1000).toFixed(2)} mm, stationary spread ${(Number(report.stationarySpreadM || 0) * 1000).toFixed(2)} mm.`;
    } catch (error) { $("#verificationReportStatus").textContent = `Report failed: ${error.message}`; }
  });
  $("#skipAccuracyForTestingBtn").addEventListener("click", async () => {
    try {
      const payload = await post("/api/camera/calibration/verification-skip", {});
      if (payload.calibration) state.calibration = payload.calibration;
      calibrationTestingBypass = true;
      accuracyVerified = true;
      calibrationWizardStep = 6;
      renderCalibrationWizard();
    } catch (error) { $("#verificationReportStatus").textContent = `Could not enter testing mode: ${error.message}`; }
  });
  $("#accuracyPointGrid").addEventListener("click", async (event) => {
    const button = event.target.closest("[data-read-accuracy]");
    if (!button) return;
    try { await readAccuracyPoint(Number(button.dataset.readAccuracy)); }
    catch (error) { $("#verificationReportStatus").textContent = `Point read failed: ${error.message}`; }
  });
  $("#finishCameraCalibrationBtn").addEventListener("click", async () => {
    try {
      $("#continuousLocalizationInput").checked = true;
      await saveCameraConfig();
      $("#cameraCalibrationSummary").textContent = calibrationTestingBypass
        ? "Testing mode: continuously localizing with unverified coordinates; physical runs show an explicit warning."
        : "Calibrated and continuously localizing in robot coordinates.";
      $("#cameraCalibrationDialog").close();
    } catch (error) { $("#verificationReportStatus").textContent = `Could not activate localization: ${error.message}`; }
  });
  const cameraTab = $("#cameraTab");
  if (cameraTab) {
    cameraTab.addEventListener("input", () => setCameraFormDirty(true));
    cameraTab.addEventListener("change", () => setCameraFormDirty(true));
  }
  // The calibration dialog lives outside #cameraTab, so its measurement
  // fields need their own dirty guard. Otherwise the 2.5-second status poll
  // redraws the saved marker map over the value while the operator types.
  const calibrationDialog = $("#cameraCalibrationDialog");
  if (calibrationDialog) {
    calibrationDialog.addEventListener("input", () => setCameraFormDirty(true));
    calibrationDialog.addEventListener("change", () => setCameraFormDirty(true));
  }
  refreshCameraDevices({ force: true }).catch(() => {});
  refreshCameraStatus().catch(() => {});
  setInterval(refreshCameraStatus, 2500);
  $("#useCurrentBtn").addEventListener("click", () => {
    setTargetInputs(state.angles);
    state.targetsInitialized = true;
  });
  $("#sendBtn").addEventListener("click", async () => {
    const speed = clamp(Number($("#speedInput").value), 1, 100);
    try {
      const payload = await post("/api/send-angles", { angles: state.targetAngles, speed });
      state.lastError = payload.ok ? null : payload.error;
      updateStatus(payload.ok ? "Sent joint targets." : "Move failed.");
    } catch (error) {
      updateStatus(`Move failed: ${error.message}`);
    }
  });

  on("scene", () => {
    renderTree();
    renderProgramEditor();
    renderEndEffectorSelect();
    renderGripperActionLabels();
    renderToolOrientationStatus();
    renderPickCalibration();
    renderToolContactCalibration();
    renderCameraStatus();
  });
  on("selection", renderInspector);
  on("drag", (target) => {
    // Live-update X/Y fields while dragging.
    const panel = $("#objectInspector");
    if (target?.position) {
      const x = panel.querySelector('input[data-field="position.x"]');
      const y = panel.querySelector('input[data-field="position.y"]');
      const z = panel.querySelector('input[data-field="position.z"]');
      if (x) x.value = target.position.x.toFixed(3);
      if (y) y.value = target.position.y.toFixed(3);
      if (z) z.value = target.position.z.toFixed(3);
    }
    updateStatus(`Moving ${target.label}: X ${target.position.x.toFixed(3)}, Y ${target.position.y.toFixed(3)}`);
  });
  on("sceneSaved", (payload) => applySceneSnapshot(payload));
  on("error", (message) => updateStatus(message));
  on("simulationDone", () => updateStatus("Simulation finished."));
  on("simulationTick", (tick) => {
    if (tick.sourceStepId && tick.sourceStepId !== state.simulationSourceStepId) {
      state.simulationSourceStepId = tick.sourceStepId;
      if (state.programWorkspaceOpen) renderProgramEditor();
    }
    const sourceIndex = state.draftSteps.findIndex((step) => step.id === tick.sourceStepId);
    const sourceStep = sourceIndex >= 0 ? state.draftSteps[sourceIndex] : null;
    const command = sourceStep ? stepLabel(sourceStep) : String(tick.stepId || "motion").replaceAll("_", " ");
    const phase = String(tick.stepId || "").split("_").slice(2).join(" ").replaceAll("_", " ");
    const message = tick.done
      ? "Simulation complete."
      : `Simulating ${command}${sourceIndex >= 0 ? ` — command ${sourceIndex + 1} of ${state.draftSteps.length}` : ""}${phase ? ` · ${phase}` : ""}`;
    if ($("#runStatus").textContent !== message) $("#runStatus").textContent = message;
  });
}

export async function loadPorts() {
  const payload = await api("/api/ports");
  const select = $("#portSelect");
  const current = select.value;
  select.innerHTML = "";
  for (const port of payload.ports) {
    const option = document.createElement("option");
    option.value = port.device;
    option.textContent = `${port.device}`;
    option.title = port.description;
    select.append(option);
  }
  if (current) select.value = current;
}
