// Shared state + tiny pub/sub. Modules mutate state through helpers here and
// subscribe to change topics instead of importing each other.

export const state = {
  // robot
  angles: [0, 0, 0, 0, 0, 0],
  renderAngles: [0, 0, 0, 0, 0, 0],
  targetAngles: [0, 0, 0, 0, 0, 0],
  renderInitialized: false,
  targetsInitialized: false,
  limits: {
    1: [-168, 168], 2: [-135, 135], 3: [-150, 150],
    4: [-145, 145], 5: [-155, 160], 6: [-180, 180],
  },
  connected: false,
  executing: false,
  lastError: null,
  port: null,

  // workcell
  parts: [],
  registeredParts: [],
  registeredBins: [],
  tagTrackRevision: 0,
  bins: [],
  supportSurfaces: [],
  supportSurfaceRevision: null,
  taughtPoints: [],
  workspaceRegions: null,
  programs: [],
  calibration: null,
  camera: null,
  coordinatePlanner: null,
  cameraStatus: null,
  cameraDevices: [],
  endEffector: "adaptive_gripper",
  endEffectors: [],
  sceneVersion: 0,

  // selection: {kind: "part"|"bin"|"robot"|"program", id}
  selection: { kind: "robot", id: null },

  // program editor
  activeProgramId: null,
  draftSteps: [],
  draftName: "Program 1",
  draftRepeatCount: 1,
  draftRunPolicy: { mode: "finite", cycleCount: 1, maxCycles: null, triggerPartId: null, stableFrames: 3, rearmAbsentMs: 1000, cooldownMs: 500, xyEnvelopeM: 0.015, yawEnvelopeDeg: 10, expectedSurfaceId: null },
  productionRuntime: { state: "disarmed", cycleCount: 0 },
  selectedProgramStepId: null,
  programInsertIndex: 0,
  programDirty: false,
  programWorkspaceOpen: false,
  editingWaypointStepId: null,
  jogTab: "joint",
  jogMode: "hold",
  jogIncrementJoint: 1,
  jogIncrementTcp: 5,
  jogIncrementRotation: 1,
  jogSpeed: 10,
  simulationSourceStepId: null,
  executionSourceStepId: null,
  lastPlan: null,
  planSource: null, // "program" | "quick"

  // simulation / physical run
  simulation: null, // {plan, startedAt, totalMs, done, heldId}
  simulatedPartPositions: new Map(),
  displayedPartPositions: new Map(),
  displayedBinPositions: new Map(),
  dragActive: false,
  physicalRunActive: false,

  // gripper visual
  gripperOpen: 0.08,
  gripperTargetOpen: 0.08,
};

const listeners = new Map();

export function on(topic, fn) {
  if (!listeners.has(topic)) listeners.set(topic, new Set());
  listeners.get(topic).add(fn);
}

export function emit(topic, payload) {
  const set = listeners.get(topic);
  if (set) for (const fn of set) fn(payload);
}

export function setSelection(kind, id) {
  state.selection = { kind, id: id ?? null };
  emit("selection");
}

export function applySceneSnapshot(payload) {
  if (!payload) return;
  state.parts = payload.parts || [];
  state.registeredParts = payload.registeredParts || state.registeredParts || [];
  state.registeredBins = payload.registeredBins || state.registeredBins || [];
  state.tagTrackRevision = payload.tagTrackRevision ?? state.tagTrackRevision;
  state.bins = payload.bins || [];
  state.supportSurfaces = payload.supportSurfaces || state.supportSurfaces || [];
  state.supportSurfaceRevision = payload.supportSurfaceRevision || state.supportSurfaceRevision;
  state.taughtPoints = payload.taughtPoints || [];
  state.workspaceRegions = payload.workspaceRegions || null;
  state.programs = payload.programs || [];
  state.calibration = payload.calibration || null;
  state.camera = payload.camera || state.camera;
  state.coordinatePlanner = payload.coordinatePlanner || state.coordinatePlanner;
  state.endEffector = payload.endEffector || "adaptive_gripper";
  state.endEffectors = payload.endEffectors || [];
  state.sceneVersion = payload.version ?? state.sceneVersion;
  // Drop selection if the object vanished.
  const sel = state.selection;
  if (sel.kind === "part" && !state.parts.some((p) => p.id === sel.id)) {
    state.selection = { kind: "robot", id: null };
  }
  if (sel.kind === "bin" && !state.bins.some((b) => b.id === sel.id)) {
    state.selection = { kind: "robot", id: null };
  }
  if (sel.kind === "point" && !state.taughtPoints.some((point) => point.id === sel.id)) {
    state.selection = { kind: "robot", id: null };
  }
  emit("scene");
  emit("selection");
}

export function applyTagTracks(payload) {
  if (!payload || payload.revision === state.tagTrackRevision) return;
  const incoming = new Map((payload.parts || []).map((part) => [part.id, part]));
  const removed = new Set(payload.removedIds || []);
  let membershipChanged = false;
  const next = [];
  for (const part of state.parts) {
    if (removed.has(part.id) && part.trackingMode === "apriltag") { membershipChanged = true; continue; }
    const update = incoming.get(part.id);
    if (update) {
      Object.assign(part, update);
      incoming.delete(part.id);
    }
    next.push(part);
  }
  for (const part of incoming.values()) { next.push(part); membershipChanged = true; }
  state.parts = next;
  const incomingBins = new Map((payload.bins || []).map((bin) => [bin.id, bin]));
  const removedBins = new Set(payload.removedBinIds || []);
  for (const bin of state.bins) {
    const update = incomingBins.get(bin.id);
    if (update) {
      const wasVisible = bin.displayVisible;
      Object.assign(bin, update);
      if (!wasVisible && bin.displayVisible) membershipChanged = true;
    } else if (removedBins.has(bin.id) && bin.trackingMode === "apriltag") {
      if (bin.displayVisible !== false) membershipChanged = true;
      bin.displayVisible = false;
      bin.poseFresh = false;
    }
  }
  state.tagTrackRevision = payload.revision;
  if (membershipChanged) emit("scene");
  else emit("tagTracks", [...(payload.parts || []), ...(payload.bins || [])]);
}

export function findPart(id) {
  return state.parts.find((p) => p.id === id) || null;
}

export function findBin(id) {
  return state.bins.find((b) => b.id === id) || null;
}

export function findPoint(id) {
  return state.taughtPoints.find((point) => point.id === id) || null;
}

export function clamp(value, min, max) {
  return Math.max(min, Math.min(max, Number(value) || 0));
}
