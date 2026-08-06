// 3D workcell viewport: robot digital twin, parts, parametric bins,
// selection + dragging, planned-path visualization, and plan simulation.

import * as THREE from "three";
import { ColladaLoader } from "/vendor/three/examples/jsm/loaders/ColladaLoader.js";
import { state, on, emit, setSelection, findPart, findBin, clamp } from "./store.js?v=37";
import { post } from "./api.js?v=28";

const ASSET_ROOT = "/vendor/mycobot_280_m5";
const GRIPPER_ROOT = "/vendor/adaptive_gripper";
const SUCTION_GRIPPER_ROOT = "/vendor/suction_gripper";

export const SCENE_METERS_TO_UNITS = 9.5;
const GRID_SIZE_UNITS = 8;
export const SCENE_BOUND_METERS = GRID_SIZE_UNITS / 2 / SCENE_METERS_TO_UNITS;
// Same stationary-frame calibration used by mycobot_kinematics.py. The CAD
// chain is expressed in its model base; parts and firmware coordinates are in
// the robot base, so the complete robot root receives this one translation.
const FIRMWARE_BASE_TRANSLATION_M = { x: 0.00182, y: 0.00130, z: 0.00762 };
const RENDER_SMOOTHING = 0.22;
const GRIPPER_VISUAL_OPEN = 0.08;
const GRIPPER_VISUAL_CLOSED = 1;
const GRIPPER_GRASP_POCKET_OPEN_Y = 0.078;
const GRIPPER_JAW_LATERAL_Z = 0.004;
const GRIPPER_JAW_HALF_SPAN_OPEN_M = 0.04;
const SUCTION_CONTACT_DISTANCE_M = 0.072;
const SUCTION_CUP_DIAMETER_M = 0.022;
const SUCTION_MOUNT_TRANSLATION_M = 0.010;
const SUCTION_LOCAL_CONTACT_DISTANCE_M = SUCTION_CONTACT_DISTANCE_M - SUCTION_MOUNT_TRANSLATION_M;
const SUCTION_FACE_CLOCKING_RAD = Math.PI / 2;

const displayAngleOffsets = [0, 90, 0, 0, 90, 0];

let viewport;
let scene;
let camera;
let renderer;
let viewportResizeObserver = null;
let robot;
let environmentGroup;
let pathGroup;
let cameraOverlayGroup;
let pickDiagnosticGroup;
let gripperFrame = null;
let flangeFrame = null;
let gripperLeftFinger = null;
let gripperRightFinger = null;
let gripperLinks = null;
let loadedEndEffector = null;
let cameraYaw = -0.78;
let cameraPitch = 0.5;
let cameraDistance = 11.4;
let isDragging = false;
let previousPointer = null;
let objectDrag = null;
let lastPickDiagnosticAt = 0;
const partGroups = new Map();
const binGroups = new Map();
const pointGroups = new Map();
const surfaceGroups = new Map();

const loader = new ColladaLoader();
const xAxis = new THREE.Vector3(1, 0, 0);
const zAxis = new THREE.Vector3(0, 0, 1);
const floorNormal = new THREE.Vector3(0, 1, 0);
const raycaster = new THREE.Raycaster();
const pointerNdc = new THREE.Vector2();
const tempQuat = new THREE.Quaternion();

function clonePosition(position) {
  return {
    x: Number(position?.x) || 0,
    y: Number(position?.y) || 0,
    z: Number(position?.z) || 0,
  };
}

function disposeGroup(group) {
  group.traverse((child) => {
    if (child.geometry) child.geometry.dispose();
    const materials = child.material ? (Array.isArray(child.material) ? child.material : [child.material]) : [];
    for (const material of materials) material.dispose?.();
  });
}

function displayedPosition(map, id, target, instant = false) {
  let current = map.get(id);
  const next = clonePosition(target);
  if (!current || instant) {
    current = next;
    map.set(id, current);
    return current;
  }
  const alpha = state.simulation ? 0.42 : 0.28;
  current.x += (next.x - current.x) * alpha;
  current.y += (next.y - current.y) * alpha;
  current.z += (next.z - current.z) * alpha;
  return current;
}

export function robotFrameToScene(position) {
  return new THREE.Vector3(
    Number(position.x) * SCENE_METERS_TO_UNITS,
    Number(position.z) * SCENE_METERS_TO_UNITS,
    -Number(position.y) * SCENE_METERS_TO_UNITS
  );
}

export function sceneToRobotFrame(vector) {
  return {
    x: vector.x / SCENE_METERS_TO_UNITS,
    y: -vector.z / SCENE_METERS_TO_UNITS,
    z: vector.y / SCENE_METERS_TO_UNITS,
  };
}

function degToRad(degrees) {
  return (degrees * Math.PI) / 180;
}

function radToDeg(radians) {
  return (radians * 180) / Math.PI;
}

function normalizeAngleDelta(delta) {
  return ((((Number(delta) || 0) + 180) % 360) + 360) % 360 - 180;
}

function interpolateAngles(a, b, t) {
  if (!a || !b) return b ? b.slice() : null;
  return b.map((value, i) => Number(a[i] || 0) + normalizeAngleDelta(Number(value || 0) - Number(a[i] || 0)) * t);
}

function rpyQuat(rpy) {
  return new THREE.Quaternion().setFromEuler(new THREE.Euler(rpy[0], rpy[1], rpy[2], "XYZ"));
}

// ---------------------------------------------------------------- model

async function loadDaePath(path) {
  const collada = await loader.loadAsync(path);
  const object = collada.scene;
  object.traverse((child) => {
    if (!child.isMesh) return;
    child.castShadow = true;
    child.receiveShadow = true;
    if (child.material) {
      const mats = Array.isArray(child.material) ? child.material : [child.material];
      mats.forEach((material) => {
        material.side = THREE.DoubleSide;
        material.needsUpdate = true;
      });
    }
  });
  return object;
}

async function createVisual(fileName, xyz, rpy) {
  const visual = new THREE.Group();
  const mesh = await loadDaePath(`${ASSET_ROOT}/${fileName}`);
  mesh.position.set(xyz[0], xyz[1], xyz[2]);
  mesh.quaternion.copy(rpyQuat(rpy));
  visual.add(mesh);
  return visual;
}

async function createGripperVisual(fileName, xyz, rpy) {
  const visual = new THREE.Group();
  const mesh = await loadDaePath(`${GRIPPER_ROOT}/${fileName}`);
  mesh.traverse((child) => {
    if (!child.isMesh) return;
    child.material = new THREE.MeshStandardMaterial({
      color: 0xf7f7f2,
      roughness: 0.72,
      metalness: 0.02,
    });
    child.castShadow = true;
    child.receiveShadow = true;
  });
  mesh.position.set(xyz[0], xyz[1], xyz[2]);
  mesh.quaternion.copy(rpyQuat(rpy));
  visual.add(mesh);
  return visual;
}

async function createSuctionGripperCadVisual(fileName, xyz, rpy) {
  const visual = new THREE.Group();
  const mesh = await loadDaePath(`${SUCTION_GRIPPER_ROOT}/${fileName}`);
  mesh.position.set(xyz[0], xyz[1], xyz[2]);
  mesh.quaternion.copy(rpyQuat(rpy));
  visual.add(mesh);
  return visual;
}

async function createGripperLink(fileName, visualOrigin) {
  const link = new THREE.Group();
  link.add(await createGripperVisual(fileName, visualOrigin, [0, 0, 0]));
  return link;
}

async function createAdaptiveGripper() {
  const gripper = new THREE.Group();
  gripper.name = "adaptive-gripper";
  gripper.userData.tcpOffset = new THREE.Vector3(0, GRIPPER_GRASP_POCKET_OPEN_Y, GRIPPER_JAW_LATERAL_Z);
  // The grasp TCP is the midpoint between the two jaw contact planes.  Keep
  // this on the modeled gripper itself so rendering and planning use the same
  // physical point instead of treating the flange origin as the grasp point.
  gripper.userData.jawCenterLocal = new THREE.Vector3(0, GRIPPER_GRASP_POCKET_OPEN_Y, GRIPPER_JAW_LATERAL_Z);
  gripper.userData.jawAxisLocal = new THREE.Vector3(1, 0, 0);
  gripper.userData.approachAxisLocal = new THREE.Vector3(0, 1, 0);
  gripperLinks = {};

  gripper.add(await createGripperVisual("gripper_base.dae", [0, 0, 0], [0, 0, 0]));

  gripperLinks.left3 = await createGripperLink("gripper_left3.dae", [0.012, 0.0025, 0]);
  gripperLinks.left3.position.set(-0.012, 0.005, 0);
  gripperLinks.left1 = await createGripperLink("gripper_left1.dae", [0.039, -0.0133, 0]);
  gripperLinks.left1.position.set(-0.027, 0.016, 0);
  gripperLinks.left3.add(gripperLinks.left1);
  gripperLinks.left2 = await createGripperLink("gripper_left2.dae", [0.005, -0.0195, 0]);
  gripperLinks.left2.position.set(-0.005, 0.027, 0);

  gripperLinks.right3 = await createGripperLink("gripper_right3.dae", [-0.012, 0.0025, 0]);
  gripperLinks.right3.position.set(0.012, 0.005, 0);
  gripperLinks.right1 = await createGripperLink("gripper_right1.dae", [-0.039, -0.0133, 0]);
  gripperLinks.right1.position.set(0.027, 0.016, 0);
  gripperLinks.right3.add(gripperLinks.right1);
  gripperLinks.right2 = await createGripperLink("gripper_right2.dae", [-0.005, -0.0195, 0]);
  gripperLinks.right2.position.set(0.005, 0.027, 0);

  gripperLeftFinger = new THREE.Group();
  gripperLeftFinger.add(gripperLinks.left3, gripperLinks.left2);
  gripperRightFinger = new THREE.Group();
  gripperRightFinger.add(gripperLinks.right3, gripperLinks.right2);
  gripper.add(gripperLeftFinger, gripperRightFinger);

  gripper.rotation.set(Math.PI / 2, Math.PI / 4, 0);
  gripper.position.set(0.0, 0.01, 0.035);
  applyGripperState();
  return gripper;
}

function makeCylinder(radiusTop, radiusBottom, height, color, roughness = 0.62, metalness = 0.08) {
  const geometry = new THREE.CylinderGeometry(radiusTop, radiusBottom, height, 36);
  const material = new THREE.MeshStandardMaterial({ color, roughness, metalness });
  const mesh = new THREE.Mesh(geometry, material);
  mesh.castShadow = true;
  mesh.receiveShadow = true;
  return mesh;
}

function makeBox(width, height, depth, color, roughness = 0.68, metalness = 0.03) {
  const geometry = new THREE.BoxGeometry(width, height, depth);
  const material = new THREE.MeshStandardMaterial({ color, roughness, metalness });
  const mesh = new THREE.Mesh(geometry, material);
  mesh.castShadow = true;
  mesh.receiveShadow = true;
  return mesh;
}

function applySuctionMountRotation(group) {
  // Net +90 degrees when looking at the outward face of J6. This is the
  // requested 180-degree correction from the prior rear-view interpretation.
  group.quaternion.setFromAxisAngle(zAxis, SUCTION_FACE_CLOCKING_RAD);
  group.quaternion.multiply(
    new THREE.Quaternion().setFromAxisAngle(xAxis, 1.579)
  );
}

function createSuctionGripperFallback() {
  const gripper = new THREE.Group();
  gripper.name = "suction-gripper";
  gripper.userData.tcpOffset = new THREE.Vector3(0, SUCTION_LOCAL_CONTACT_DISTANCE_M, 0);
  gripper.userData.jawCenterLocal = gripper.userData.tcpOffset.clone();
  gripper.userData.jawAxisLocal = new THREE.Vector3(1, 0, 0);
  gripper.userData.approachAxisLocal = new THREE.Vector3(0, 1, 0);

  const wristPlate = makeCylinder(0.018, 0.018, 0.012, 0xd8dde2, 0.42, 0.18);
  wristPlate.rotation.x = Math.PI / 2;
  wristPlate.position.y = 0.008;

  const clamp = makeBox(0.035, 0.024, 0.024, 0xf7f7f2, 0.7, 0.02);
  clamp.position.set(0, 0.027, 0.002);

  const pumpCap = makeCylinder(0.014, 0.014, 0.023, 0xf7f7f2, 0.7, 0.02);
  pumpCap.rotation.x = Math.PI / 2;
  pumpCap.position.set(0, 0.043, -0.018);

  const pumpBody = makeCylinder(0.0115, 0.0115, 0.052, 0xf7f7f2, 0.66, 0.02);
  pumpBody.rotation.x = Math.PI / 2;
  pumpBody.position.set(0, 0.047, -0.031);

  const lowerBand = makeCylinder(0.012, 0.012, 0.012, 0xc9d0d5, 0.5, 0.14);
  lowerBand.rotation.x = Math.PI / 2;
  lowerBand.position.set(0, 0.047, -0.058);

  const nozzle = makeCylinder(0.005, 0.006, 0.011, 0xeff1f1, 0.58, 0.05);
  nozzle.rotation.x = Math.PI / 2;
  nozzle.position.set(0, 0.047, -0.071);

  const hose = new THREE.Mesh(
    new THREE.TorusGeometry(0.016, 0.0022, 8, 32, Math.PI * 0.72),
    new THREE.MeshStandardMaterial({ color: 0xd7dee0, roughness: 0.72, metalness: 0.0 })
  );
  hose.rotation.set(0.28, 0.05, -Math.PI / 2);
  hose.position.set(0.002, 0.047, -0.08);
  hose.castShadow = true;
  hose.receiveShadow = true;

  const sideScrew = makeCylinder(0.0022, 0.0022, 0.0016, 0x59616b, 0.35, 0.45);
  sideScrew.rotation.z = Math.PI / 2;
  sideScrew.position.set(0.0128, 0.027, 0.014);

  const cup = makeCylinder(
    SUCTION_CUP_DIAMETER_M / 2, SUCTION_CUP_DIAMETER_M / 2,
    0.022, 0x1f2937, 0.78, 0.01,
  );
  cup.position.set(0, 0.051, 0);

  gripper.add(wristPlate, clamp, pumpCap, pumpBody, lowerBand, nozzle, hose, sideScrew, cup);
  applySuctionMountRotation(gripper);
  gripper.position.set(0, 0, SUCTION_MOUNT_TRANSLATION_M);
  return gripper;
}

async function createSuctionGripper() {
  try {
    const gripper = new THREE.Group();
    gripper.name = "suction-gripper";
    gripper.userData.tcpOffset = new THREE.Vector3(0, SUCTION_LOCAL_CONTACT_DISTANCE_M, 0);
    gripper.userData.jawCenterLocal = gripper.userData.tcpOffset.clone();
    gripper.userData.jawAxisLocal = new THREE.Vector3(1, 0, 0);
    gripper.userData.approachAxisLocal = new THREE.Vector3(0, 1, 0);
    gripper.add(await createSuctionGripperCadVisual("pump_head.dae", [0, -0.008, 0], [0, 0, 0]));
    // The installed 22 mm cup is larger than the published 20 mm part. Draw
    // the measured compliant extension explicitly and end it at the TCP.
    const cup = makeCylinder(SUCTION_CUP_DIAMETER_M / 2, SUCTION_CUP_DIAMETER_M / 2, 0.022, 0x1f2937, 0.78, 0.01);
    cup.position.set(0, 0.051, 0);
    gripper.add(cup);
    applySuctionMountRotation(gripper);
    // Official fixed joint: flange -> pump head is +10 mm in flange Z,
    // followed by the 1.579-radian mount rotation. The remaining 62 mm ends
    // at the measured 72 mm flange-to-contact TCP.
    gripper.position.set(0, 0, SUCTION_MOUNT_TRANSLATION_M);
    return gripper;
  } catch (error) {
    console.info("Using procedural suction head model; official pump_head.dae was not available.", error);
    return createSuctionGripperFallback();
  }
}

async function createEndEffector(endEffector) {
  gripperLeftFinger = null;
  gripperRightFinger = null;
  gripperLinks = null;
  if (endEffector === "suction_gripper") return createSuctionGripper();
  return createAdaptiveGripper();
}

export async function syncEndEffector(force = false) {
  if (!flangeFrame) return;
  const nextEndEffector = state.endEffector || "adaptive_gripper";
  if (!force && loadedEndEffector === nextEndEffector && gripperFrame) return;
  if (gripperFrame) {
    flangeFrame.remove(gripperFrame);
    disposeGroup(gripperFrame);
    gripperFrame = null;
  }
  gripperFrame = await createEndEffector(nextEndEffector);
  loadedEndEffector = nextEndEffector;
  flangeFrame.add(gripperFrame);
  flangeFrame.updateMatrixWorld(true);
}

function makeJointFrame(parent, xyz, rpy) {
  const frame = new THREE.Group();
  frame.position.set(xyz[0], xyz[1], xyz[2]);
  frame.userData.baseQuat = rpyQuat(rpy);
  frame.userData.physicalCorrection = new THREE.Quaternion();
  frame.quaternion.copy(frame.userData.baseQuat);
  parent.add(frame);
  return frame;
}

async function createOfficialMyCobot() {
  const model = new THREE.Group();
  model.add(await createVisual("G_base.dae", [0, 0, -0.03], [0, 0, 0]));
  // Official pump URDF: the pneumatic box is fixed to g_base, never J1.
  // Its mass therefore stays out of moving-arm payload calculations.
  try {
    model.add(await createSuctionGripperCadVisual(
      "pump_box.dae", [0, -0.15, 0], [Math.PI / 2, Math.PI, 0]
    ));
  } catch (error) {
    console.info("Official base pump-box CAD was not available.", error);
  }

  const joint1 = makeJointFrame(model, [0, 0, 0], [0, 0, 0]);
  joint1.add(await createVisual("joint1.dae", [0, 0, 0], [0, 0, -1.5708]));
  const joint2 = makeJointFrame(joint1, [0, 0, 0.13156], [0, 0, 0]);
  joint2.add(await createVisual("joint2.dae", [0, 0, -0.06096], [0, 0, -1.5708]));
  const joint3 = makeJointFrame(joint2, [0, 0, 0], [0, 1.5708, -1.5708]);
  joint3.userData.physicalCorrection.setFromAxisAngle(zAxis, -Math.PI / 2);
  joint3.add(await createVisual("joint3.dae", [0, 0, 0.03256], [0, -1.5708, 0]));
  const joint4 = makeJointFrame(joint3, [-0.1104, 0, 0], [0, 0, 0]);
  joint4.add(await createVisual("joint4.dae", [0, 0, 0.03056], [0, -1.5708, 0]));
  const joint5 = makeJointFrame(joint4, [-0.096, 0, 0.06462], [0, 0, -1.5708]);
  joint5.add(await createVisual("joint5.dae", [0, 0, -0.03356], [-1.5708, 0, 0]));
  const joint6 = makeJointFrame(joint5, [0, -0.07318, 0], [1.5708, -1.5708, 0]);
  joint6.userData.physicalCorrection.setFromAxisAngle(zAxis, Math.PI / 2);
  joint6.add(await createVisual("joint6.dae", [0, 0, -0.038], [0, 0, 0]));
  const flange = makeJointFrame(joint6, [0, 0.0456, 0], [-1.5708, 0, 0]);
  flange.add(await createVisual("joint7.dae", [0, 0, -0.012], [0, 0, 0]));
  flangeFrame = flange;
  await syncEndEffector();

  model.userData.jointFrames = [joint1, joint3, joint4, joint5, joint6, flange];

  const zUpToYUp = new THREE.Group();
  zUpToYUp.rotation.x = -Math.PI / 2;
  zUpToYUp.add(model);
  const fitted = new THREE.Group();
  fitted.add(zUpToYUp);
  fitted.updateMatrixWorld(true);
  const box = new THREE.Box3().setFromObject(fitted);
  const size = box.getSize(new THREE.Vector3());
  // Link translations and loaded CAD assets are expressed in meters.  Using
  // an independent bounding-box fit here used to scale the robot differently
  // from parts, paths, and TCP targets; the resulting error grew with reach
  // and made a correct pick look centimeters off in the digital twin.
  const scale = SCENE_METERS_TO_UNITS;
  zUpToYUp.scale.setScalar(scale);
  fitted.position.copy(robotFrameToScene(FIRMWARE_BASE_TRANSLATION_M));
  fitted.userData.jointFrames = model.userData.jointFrames;
  fitted.userData.modelScale = scale;
  fitted.userData.unscaledBounds = {
    min: box.min.toArray(),
    max: box.max.toArray(),
    size: size.toArray(),
  };
  return fitted;
}

// ------------------------------------------------------------ environment

function partColor(part) {
  if (typeof part.color === "string" && /^#[0-9a-fA-F]{6}$/.test(part.color)) return part.color;
  return "#2f80ed";
}

function makePartMesh(part) {
  const size = part.size || { x: 0.05, y: 0.05, z: 0.05 };
  const color = partColor(part);
  const cameraSource = part.source === "camera";
  const stale = Boolean(part.stale);
  const degraded = part.trackingState === "degraded_recent";
  const u = SCENE_METERS_TO_UNITS;
  const material = new THREE.MeshStandardMaterial({
    color, roughness: 0.62, metalness: 0.05,
    transparent: cameraSource,
    opacity: cameraSource ? 0.86 : 1,
  });
  let mesh;
  if (part.type === "sphere") {
    mesh = new THREE.Mesh(new THREE.SphereGeometry(
      Math.max(size.x, size.y, size.z) * u * 0.5, 32, 18), material);
  } else if (part.type === "cylinder" || part.type === "circle") {
    // circle = flat disc; same geometry, height comes from size.z
    mesh = new THREE.Mesh(new THREE.CylinderGeometry(
      Math.max(size.x, size.y) * u * 0.5,
      Math.max(size.x, size.y) * u * 0.5,
      size.z * u, 36), material);
  } else if (part.type === "open-box") {
    // Box with an open lid: floor + 4 thin walls.
    mesh = new THREE.Group();
    const t = Math.min(0.005, size.x / 4, size.y / 4) * u;
    const L = size.x * u, W = size.y * u, H = size.z * u;
    const floor = new THREE.Mesh(new THREE.BoxGeometry(L, t, W), material);
    floor.position.y = -H / 2 + t / 2;
    const north = new THREE.Mesh(new THREE.BoxGeometry(L, H, t), material);
    north.position.z = -(W - t) / 2;
    const south = new THREE.Mesh(new THREE.BoxGeometry(L, H, t), material);
    south.position.z = (W - t) / 2;
    const east = new THREE.Mesh(new THREE.BoxGeometry(t, H, W - 2 * t), material);
    east.position.x = (L - t) / 2;
    const west = new THREE.Mesh(new THREE.BoxGeometry(t, H, W - 2 * t), material);
    west.position.x = -(L - t) / 2;
    mesh.add(floor, north, south, east, west);
  } else {
    // box and rectangle: rectangle is just a flat box (thin size.z)
    mesh = new THREE.Mesh(new THREE.BoxGeometry(
      size.x * u, size.z * u, size.y * u), material);
  }
  mesh.traverse?.((child) => {
    if (child.isMesh) {
      child.castShadow = true;
      child.receiveShadow = true;
      if (degraded && child.material?.emissive) {
        child.material.emissive.set("#f59e0b");
        child.material.emissiveIntensity = 0.35;
      } else if (stale && child.material) {
        child.material.opacity = Math.min(Number(child.material.opacity || 1), 0.5);
      }
    }
  });
  if (mesh.isMesh) { mesh.castShadow = true; mesh.receiveShadow = true; }

  const selected = state.selection.kind === "part" && state.selection.id === part.id;
  const position = state.simulatedPartPositions.get(part.id) || part.position;
  const scenePosition = robotFrameToScene(position);

  const ring = new THREE.Mesh(
    new THREE.RingGeometry(0.09, selected ? 0.125 : 0.105, 40),
    new THREE.MeshBasicMaterial({
      color: selected ? "#1d4ed8" : cameraSource ? "#0f766e" : color,
      transparent: true, opacity: selected ? 0.8 : cameraSource ? 0.55 : 0.35, side: THREE.DoubleSide,
    })
  );
  ring.rotation.x = -Math.PI / 2;
  ring.position.set(0, 0.006 - scenePosition.y, 0);

  const group = new THREE.Group();
  group.userData = { kind: "part", id: part.id, ring };
  group.position.copy(scenePosition);
  group.rotation.y = -degToRad(Number(part.orientationDeg || 0));
  group.add(mesh, ring);
  return group;
}

function makeBinMesh(bin) {
  // Bins render as shallow trays so Z and H are visible in the workcell.
  const color = bin.color && /^#[0-9a-fA-F]{6}$/.test(bin.color) ? bin.color : "#f59e0b";
  const u = SCENE_METERS_TO_UNITS;
  const L = bin.outer.x * u, W = bin.outer.y * u, H = Math.max(0.01, Number(bin.outer.z || 0.02)) * u;
  const wall = Math.max(0.006, Number(bin.wallThickness || 0.008)) * u;
  const floorThickness = Math.min(H, Math.max(0.018, wall * 0.75));
  const selected = state.selection.kind === "bin" && state.selection.id === bin.id;
  const material = new THREE.MeshStandardMaterial({
    color, roughness: 0.65, metalness: 0.03,
    transparent: true, opacity: selected ? 0.95 : 0.75,
  });

  const group = new THREE.Group();
  group.userData = { kind: "bin", id: bin.id };

  const floor = new THREE.Mesh(new THREE.BoxGeometry(L, floorThickness, W), material);
  floor.position.y = floorThickness / 2;
  floor.receiveShadow = true;
  const north = new THREE.Mesh(new THREE.BoxGeometry(L, H, wall), material);
  north.position.set(0, H / 2, -(W - wall) / 2);
  const south = new THREE.Mesh(new THREE.BoxGeometry(L, H, wall), material);
  south.position.set(0, H / 2, (W - wall) / 2);
  const east = new THREE.Mesh(new THREE.BoxGeometry(wall, H, Math.max(wall, W - 2 * wall)), material);
  east.position.set((L - wall) / 2, H / 2, 0);
  const west = new THREE.Mesh(new THREE.BoxGeometry(wall, H, Math.max(wall, W - 2 * wall)), material);
  west.position.set(-(L - wall) / 2, H / 2, 0);
  for (const wallMesh of [north, south, east, west]) {
    wallMesh.castShadow = true;
    wallMesh.receiveShadow = true;
  }
  group.add(floor, north, south, east, west);

  const outline = new THREE.LineSegments(
    new THREE.EdgesGeometry(new THREE.PlaneGeometry(L, W)),
    new THREE.LineBasicMaterial({ color: selected ? "#1d4ed8" : color })
  );
  outline.rotation.x = -Math.PI / 2;
  outline.position.y = H + 0.006;
  group.add(outline);

  // Interior drop boundary inset by the bin's inset value.
  const geometry = bin.geometry || {};
  const inner = geometry.interior || {
    x: Math.max(0.01, bin.outer.x - 2 * bin.wallThickness),
    y: Math.max(0.01, bin.outer.y - 2 * bin.wallThickness),
  };
  const boundary = new THREE.LineSegments(
    new THREE.EdgesGeometry(new THREE.PlaneGeometry(inner.x * u, inner.y * u)),
    new THREE.LineBasicMaterial({ color: selected ? "#1d4ed8" : "#0f766e" })
  );
  boundary.rotation.x = -Math.PI / 2;
  boundary.position.y = H + 0.012;
  group.add(boundary);

  const scenePosition = robotFrameToScene(bin.position);
  group.position.set(scenePosition.x, bin.position.z * u, scenePosition.z);
  group.rotation.y = -degToRad(Number(bin.orientationDeg || 0));
  return group;
}

function makePointMesh(point) {
  const selected = state.selection.kind === "point" && state.selection.id === point.id;
  const group = new THREE.Group();
  group.userData = { kind: "point", id: point.id };
  const material = new THREE.MeshStandardMaterial({
    color: selected ? 0x1d4ed8 : 0x0ea5e9,
    emissive: selected ? 0x0b3b91 : 0x062b3b,
    roughness: 0.45,
  });
  const marker = new THREE.Mesh(new THREE.SphereGeometry(selected ? 0.055 : 0.042, 18, 12), material);
  const stem = new THREE.Mesh(
    new THREE.CylinderGeometry(0.008, 0.008, 0.16, 10),
    new THREE.MeshBasicMaterial({ color: selected ? 0x1d4ed8 : 0x0ea5e9 })
  );
  stem.position.y = -0.08;
  group.add(marker, stem);
  group.position.copy(robotFrameToScene(point.tcpPoseM?.position || { x: 0, y: 0, z: 0 }));
  return group;
}

function makeSupportSurfaceMesh(surface) {
  const group = new THREE.Group();
  group.userData = { supportSurfaceId: surface.id };
  if (surface.id === "surface-table" || surface.enabled === false) return group;
  const u = SCENE_METERS_TO_UNITS;
  const height = Math.max(0.002, Number(surface.topZ || 0)) * u;
  const material = new THREE.MeshStandardMaterial({
    color: surface.color || "#8aa4bd", roughness: 0.82,
    transparent: true, opacity: 0.45,
  });
  const mesh = new THREE.Mesh(new THREE.BoxGeometry(
    Number(surface.size?.x || 0.2) * u,
    height,
    Number(surface.size?.y || 0.2) * u,
  ), material);
  mesh.position.y = height / 2;
  mesh.receiveShadow = true;
  const center = robotFrameToScene({
    x: Number(surface.center?.x || 0), y: Number(surface.center?.y || 0), z: 0,
  });
  group.position.set(center.x, 0, center.z);
  group.add(mesh);
  return group;
}

function makeCameraOverlay() {
  const group = new THREE.Group();
  const cameraPose = state.calibration?.cameraToRobot;
  if (cameraPose?.position) {
    const marker = new THREE.Mesh(
      new THREE.BoxGeometry(0.18, 0.10, 0.12),
      new THREE.MeshStandardMaterial({ color: 0x2563eb, roughness: 0.55, metalness: 0.05 })
    );
    marker.position.copy(robotFrameToScene(cameraPose.position));
    group.add(marker);
    const p = marker.position.clone();
    const points = [
      p, new THREE.Vector3(p.x - 0.45, 0.03, p.z - 0.35),
      p, new THREE.Vector3(p.x + 0.45, 0.03, p.z - 0.35),
      p, new THREE.Vector3(p.x + 0.45, 0.03, p.z + 0.35),
      p, new THREE.Vector3(p.x - 0.45, 0.03, p.z + 0.35),
    ];
    group.add(new THREE.LineSegments(
      new THREE.BufferGeometry().setFromPoints(points),
      new THREE.LineBasicMaterial({ color: 0x2563eb, transparent: true, opacity: 0.7 })
    ));
  }
  const bounds = state.camera?.workspaceBounds;
  if (bounds) {
    const corners = [
      robotFrameToScene({ x: bounds.xMin, y: bounds.yMin, z: 0.004 }),
      robotFrameToScene({ x: bounds.xMax, y: bounds.yMin, z: 0.004 }),
      robotFrameToScene({ x: bounds.xMax, y: bounds.yMax, z: 0.004 }),
      robotFrameToScene({ x: bounds.xMin, y: bounds.yMax, z: 0.004 }),
      robotFrameToScene({ x: bounds.xMin, y: bounds.yMin, z: 0.004 }),
    ];
    group.add(new THREE.Line(
      new THREE.BufferGeometry().setFromPoints(corners),
      new THREE.LineBasicMaterial({ color: 0x0f766e, linewidth: 2 })
    ));
  }
  return group;
}

export function renderEnvironment() {
  if (!environmentGroup) return;
  for (const group of partGroups.values()) disposeGroup(group);
  for (const group of binGroups.values()) disposeGroup(group);
  for (const group of pointGroups.values()) disposeGroup(group);
  for (const group of surfaceGroups.values()) disposeGroup(group);
  environmentGroup.clear();
  partGroups.clear();
  binGroups.clear();
  pointGroups.clear();
  surfaceGroups.clear();

  for (const surface of state.supportSurfaces || []) {
    const group = makeSupportSurfaceMesh(surface);
    surfaceGroups.set(surface.id, group);
    environmentGroup.add(group);
  }

  const partIds = new Set(state.parts.map((part) => part.id));
  const binIds = new Set(state.bins.map((bin) => bin.id));
  for (const id of state.displayedPartPositions.keys()) {
    if (!partIds.has(id)) state.displayedPartPositions.delete(id);
  }
  for (const id of state.displayedBinPositions.keys()) {
    if (!binIds.has(id)) state.displayedBinPositions.delete(id);
  }

  for (const bin of state.bins) {
    const group = makeBinMesh(bin);
    binGroups.set(bin.id, group);
    environmentGroup.add(group);
  }
  for (const part of state.parts) {
    const group = makePartMesh(part);
    partGroups.set(part.id, group);
    environmentGroup.add(group);
  }
  for (const point of state.taughtPoints) {
    const group = makePointMesh(point);
    pointGroups.set(point.id, group);
    environmentGroup.add(group);
  }
  if (cameraOverlayGroup) disposeGroup(cameraOverlayGroup);
  cameraOverlayGroup = makeCameraOverlay();
  environmentGroup.add(cameraOverlayGroup);
  updateEnvironmentTransforms(true);
}

function applyPartTransform(part, instant = false) {
  const group = partGroups.get(part.id);
  if (!group) return;
  const isDragged = state.dragActive && objectDrag?.kind === "part" && objectDrag.id === part.id;
  const target = state.simulatedPartPositions.get(part.id) || part.position;
  const display = displayedPosition(state.displayedPartPositions, part.id, target, instant || isDragged);
  const scenePosition = robotFrameToScene(display);
  group.position.copy(scenePosition);
  group.rotation.y = -degToRad(Number(part.orientationDeg || 0));
  if (group.userData.ring) group.userData.ring.position.set(0, 0.006 - scenePosition.y, 0);
}

function applyBinTransform(bin, instant = false) {
  const group = binGroups.get(bin.id);
  if (!group) return;
  const isDragged = state.dragActive && objectDrag?.kind === "bin" && objectDrag.id === bin.id;
  const display = displayedPosition(state.displayedBinPositions, bin.id, bin.position, instant || isDragged);
  const scenePosition = robotFrameToScene(display);
  group.position.set(scenePosition.x, display.z * SCENE_METERS_TO_UNITS, scenePosition.z);
  group.rotation.y = -degToRad(Number(bin.orientationDeg || 0));
}

function updateEnvironmentTransforms(instant = false) {
  for (const bin of state.bins) applyBinTransform(bin, instant);
  for (const part of state.parts) applyPartTransform(part, instant);
}

export function renderPlanPath(plan) {
  if (!pathGroup) return;
  pathGroup.clear();
  if (!plan?.steps?.length) return;
  const failedStateIds = new Set(
    (plan.coordinatePreview?.states || [])
      .filter((stateResult) => stateResult?.ok === false)
      .map((stateResult) => stateResult.stateId)
  );
  const pointRecords = plan.steps
    .filter((step) => stepPose(step))
    .map((step) => ({ step, point: robotFrameToScene(stepPose(step)) }));
  if (pointRecords.length >= 2) {
    const line = new THREE.Line(
      new THREE.BufferGeometry().setFromPoints(pointRecords.map((item) => item.point)),
      new THREE.LineBasicMaterial({ color: plan.coordinatePreview?.ok === false ? 0xf97316 : 0xf59e0b })
    );
    pathGroup.add(line);
  }
  for (const { step, point } of pointRecords) {
    const failed = failedStateIds.has(step.stateId);
    const dot = new THREE.Mesh(
      new THREE.SphereGeometry(failed ? 0.024 : 0.018, 10, 8),
      new THREE.MeshBasicMaterial({ color: failed ? 0xdc2626 : 0xf59e0b })
    );
    dot.position.copy(point);
    pathGroup.add(dot);
  }
  const flangePoints = plan.steps
    .filter((s) => s.targetFlangePoseM)
    .map((s) => robotFrameToScene(s.targetFlangePoseM));
  for (const p of flangePoints) {
    const marker = new THREE.Mesh(
      new THREE.BoxGeometry(0.035, 0.035, 0.035),
      new THREE.MeshBasicMaterial({ color: 0x2563eb, transparent: true, opacity: 0.8 })
    );
    marker.position.copy(p);
    pathGroup.add(marker);
  }
}

// ----------------------------------------------------------- interaction

function updatePointerNdc(event) {
  const rect = viewport.getBoundingClientRect();
  pointerNdc.x = ((event.clientX - rect.left) / rect.width) * 2 - 1;
  pointerNdc.y = -((event.clientY - rect.top) / rect.height) * 2 + 1;
}

function findSelectableGroup(object3d) {
  let current = object3d;
  while (current) {
    if (current.userData?.kind) return current;
    current = current.parent;
  }
  return null;
}

function pickEnvironmentObject(event) {
  if (!environmentGroup) return null;
  updatePointerNdc(event);
  raycaster.setFromCamera(pointerNdc, camera);
  const hits = raycaster.intersectObjects(environmentGroup.children, true);
  for (const hit of hits) {
    const group = findSelectableGroup(hit.object);
    if (group) return group;
  }
  return null;
}

function startObjectDrag(event) {
  if (state.simulation || state.physicalRunActive) return false;
  const group = pickEnvironmentObject(event);
  if (!group) return false;
  const { kind, id } = group.userData;
  setSelection(kind, id);
  const selectedPart = kind === "part" ? findPart(id) : null;
  if (kind === "point" || selectedPart?.trackingMode === "apriltag") {
    // Taught points and camera-authoritative tagged parts can be selected, but
    // dragging them would create a fake pose that immediately snaps back.
    event.preventDefault();
    return true;
  }
  state.dragActive = true;
  const plane = new THREE.Plane(floorNormal, -group.position.y);
  objectDrag = { kind, id, group, plane, offset: new THREE.Vector3(), moved: false };
  const point = intersectDragPlane(event);
  if (point) objectDrag.offset.copy(group.position).sub(point);
  viewport.setPointerCapture(event.pointerId);
  viewport.style.cursor = "grabbing";
  event.preventDefault();
  return true;
}

function intersectDragPlane(event) {
  if (!objectDrag) return null;
  updatePointerNdc(event);
  raycaster.setFromCamera(pointerNdc, camera);
  const point = new THREE.Vector3();
  return raycaster.ray.intersectPlane(objectDrag.plane, point) ? point : null;
}

function updateObjectDrag(event) {
  const point = intersectDragPlane(event);
  if (!point) return;
  const target = objectDrag.kind === "part" ? findPart(objectDrag.id) : findBin(objectDrag.id);
  if (!target) return;
  const scenePoint = point.add(objectDrag.offset);
  const robotPoint = sceneToRobotFrame(scenePoint);
  target.position.x = clamp(robotPoint.x, -SCENE_BOUND_METERS, SCENE_BOUND_METERS);
  target.position.y = clamp(robotPoint.y, -SCENE_BOUND_METERS, SCENE_BOUND_METERS);
  objectDrag.moved = true;
  const displayMap = objectDrag.kind === "part" ? state.displayedPartPositions : state.displayedBinPositions;
  displayMap.set(target.id, clonePosition(target.position));
  objectDrag.group.position.copy(robotFrameToScene({
    x: target.position.x, y: target.position.y,
    z: objectDrag.kind === "bin" ? target.position.z : target.position.z,
  }));
  if (objectDrag.kind === "bin") {
    objectDrag.group.position.y = target.position.z * SCENE_METERS_TO_UNITS;
  }
  emit("drag", target);
}

async function commitObjectDrag() {
  const drag = objectDrag;
  objectDrag = null;
  state.dragActive = false;
  viewport.style.cursor = "";
  isDragging = false;
  previousPointer = null;
  if (!drag?.moved) return;
  const target = drag.kind === "part" ? findPart(drag.id) : findBin(drag.id);
  if (!target) return;
  if (drag.kind === "bin") {
    target.positionStatus = "simulation_only";
    target.positionSource = "viewport_drag";
  }
  try {
    const endpoint = drag.kind === "part" ? "/api/scene/part" : "/api/scene/bin";
    const payload = await post(endpoint, target);
    emit("sceneSaved", payload);
  } catch (error) {
    emit("error", `Move failed: ${error.message}`);
  }
}

// ------------------------------------------------------------ simulation

function applyAnglesToRobotFrames(angles) {
  if (!robot?.userData.jointFrames) return;
  robot.userData.jointFrames.forEach((frame, index) => {
    frame.quaternion.copy(frame.userData.physicalCorrection);
    frame.quaternion.multiply(frame.userData.baseQuat);
    const displayAngle = Number(angles[index] || 0) + displayAngleOffsets[index];
    tempQuat.setFromAxisAngle(zAxis, degToRad(displayAngle));
    frame.quaternion.multiply(tempQuat);
  });
  robot.updateMatrixWorld(true);
}

function applyGripperState() {
  state.gripperOpen += (state.gripperTargetOpen - state.gripperOpen) * 0.16;
  if (!gripperLinks?.left3 || !gripperLinks?.left2 || !gripperLinks?.left1
      || !gripperLinks?.right3 || !gripperLinks?.right2 || !gripperLinks?.right1) return;
  const opening = Math.max(0, Math.min(1, state.gripperOpen));
  const q = -0.58 * opening;
  gripperLinks.left3.rotation.z = q;
  gripperLinks.left2.rotation.z = q;
  gripperLinks.left1.rotation.z = -q;
  gripperLinks.right3.rotation.z = -q;
  gripperLinks.right2.rotation.z = -q;
  gripperLinks.right1.rotation.z = q;
}

function getGripperTcpScenePosition() {
  if (!gripperFrame) return new THREE.Vector3();
  robot?.updateMatrixWorld(true);
  const baseOffset = (gripperFrame.userData.tcpOffset || new THREE.Vector3(0, GRIPPER_GRASP_POCKET_OPEN_Y, GRIPPER_JAW_LATERAL_Z)).clone();
  const correction = state.coordinatePlanner?.toolProfiles?.[state.endEffector]?.tcpCorrectionLocalM || {};
  baseOffset.add(new THREE.Vector3(Number(correction.x || 0), Number(correction.y || 0), Number(correction.z || 0)));
  // Closing changes finger aperture, not the configured jaw-center TCP.
  return gripperFrame.localToWorld(baseOffset);
}

function gripperDirectionInRobot(localDirection) {
  if (!gripperFrame) return null;
  const worldQuat = gripperFrame.getWorldQuaternion(new THREE.Quaternion());
  const sceneDirection = localDirection.clone().applyQuaternion(worldQuat).normalize();
  return new THREE.Vector3(sceneDirection.x, -sceneDirection.z, sceneDirection.y).normalize();
}

function renderedFingerTipRobot(link, leftSide) {
  if (!link || !gripperFrame) return null;
  let furthestY = -Infinity;
  const candidates = [];
  link.traverse((child) => {
    const position = child.geometry?.attributes?.position;
    if (!position) return;
    for (let index = 0; index < position.count; index += 1) {
      const local = new THREE.Vector3().fromBufferAttribute(position, index);
      const inGripper = gripperFrame.worldToLocal(child.localToWorld(local));
      if (inGripper.y > furthestY + 0.0005) {
        furthestY = inGripper.y;
        candidates.length = 0;
        candidates.push(inGripper.clone());
      } else if (inGripper.y >= furthestY - 0.0005) {
        candidates.push(inGripper.clone());
      }
    }
  });
  if (!candidates.length) return null;
  // Select the inward surface of each distal fingertip.
  const selected = candidates.reduce((best, point) => {
    if (!best) return point;
    return leftSide ? (point.x > best.x ? point : best) : (point.x < best.x ? point : best);
  }, null);
  return sceneToRobotFrame(gripperFrame.localToWorld(selected.clone()));
}

function ensurePickDiagnostics() {
  if (pickDiagnosticGroup) return;
  pickDiagnosticGroup = new THREE.Group();
  pickDiagnosticGroup.name = "pick-diagnostics";
  const marker = (name, color, radius = 0.035) => {
    const mesh = new THREE.Mesh(
      new THREE.SphereGeometry(radius, 14, 10),
      new THREE.MeshBasicMaterial({ color, depthTest: false, transparent: true, opacity: 0.92 })
    );
    mesh.name = name;
    mesh.renderOrder = 20;
    pickDiagnosticGroup.add(mesh);
    return mesh;
  };
  pickDiagnosticGroup.userData.objectCenter = marker("object-center", 0xec4899, 0.041);
  pickDiagnosticGroup.userData.requestedTcp = marker("requested-tcp", 0xf59e0b, 0.034);
  pickDiagnosticGroup.userData.renderedJaw = marker("rendered-jaw-center", 0x06b6d4, 0.029);
  pickDiagnosticGroup.userData.flange = marker("flange-origin", 0x2563eb, 0.025);
  pickDiagnosticGroup.userData.approach = new THREE.ArrowHelper(
    new THREE.Vector3(0, -1, 0), new THREE.Vector3(), 0.5, 0x16a34a, 0.11, 0.06
  );
  pickDiagnosticGroup.userData.jaw = new THREE.ArrowHelper(
    new THREE.Vector3(1, 0, 0), new THREE.Vector3(), 0.5, 0x7c3aed, 0.11, 0.06
  );
  pickDiagnosticGroup.add(pickDiagnosticGroup.userData.approach, pickDiagnosticGroup.userData.jaw);
  pickDiagnosticGroup.visible = false;
  scene.add(pickDiagnosticGroup);
}

function updatePickDiagnostics(current, targetTcp) {
  ensurePickDiagnostics();
  if (!current || !targetTcp || !gripperFrame || !flangeFrame) {
    pickDiagnosticGroup.visible = false;
    return null;
  }
  robot?.updateMatrixWorld(true);
  const renderedJawScene = getGripperTcpScenePosition();
  const flangeScene = flangeFrame.getWorldPosition(new THREE.Vector3());
  const renderedJaw = sceneToRobotFrame(renderedJawScene);
  const renderedFlange = sceneToRobotFrame(flangeScene);
  const objectCenter = current.grasp?.objectCenter || current.grasp?.graspPoint || targetTcp;
  const approach = gripperDirectionInRobot(gripperFrame.userData.approachAxisLocal || new THREE.Vector3(0, 1, 0));
  const jaw = gripperDirectionInRobot(gripperFrame.userData.jawAxisLocal || new THREE.Vector3(1, 0, 0));
  const verticalAlignment = clamp(-(approach?.z ?? 0), -1, 1);
  const verticalErrorDeg = radToDeg(Math.acos(verticalAlignment));
  const targetErrorMm = Math.hypot(
    renderedJaw.x - Number(targetTcp.x || 0),
    renderedJaw.y - Number(targetTcp.y || 0),
    renderedJaw.z - Number(targetTcp.z || 0)
  ) * 1000;
  const objectXyErrorMm = Math.hypot(
    renderedJaw.x - Number(objectCenter.x || 0),
    renderedJaw.y - Number(objectCenter.y || 0)
  ) * 1000;
  const leftFingerTip = renderedFingerTipRobot(gripperLinks?.left1, true);
  const rightFingerTip = renderedFingerTipRobot(gripperLinks?.right1, false);
  let renderedFingerSpanMm = null;
  let fingerMidpointErrorMm = null;
  let fingerMidpointXyErrorMm = null;
  let fingerTipAxialOffsetMm = null;
  let fingerMidpoint = null;
  let renderedFingertipLowZ = null;
  let renderedFingerOverlapM = null;
  let renderedTableClearanceM = null;
  const heightModel = current.grasp?.heightModel || null;
  if (leftFingerTip && rightFingerTip) {
    renderedFingerSpanMm = Math.hypot(
      leftFingerTip.x - rightFingerTip.x,
      leftFingerTip.y - rightFingerTip.y,
      leftFingerTip.z - rightFingerTip.z
    ) * 1000;
    const midpoint = {
      x: (leftFingerTip.x + rightFingerTip.x) / 2,
      y: (leftFingerTip.y + rightFingerTip.y) / 2,
      z: (leftFingerTip.z + rightFingerTip.z) / 2,
    };
    fingerMidpoint = midpoint;
    fingerMidpointErrorMm = Math.hypot(
      midpoint.x - renderedJaw.x,
      midpoint.y - renderedJaw.y,
      midpoint.z - renderedJaw.z
    ) * 1000;
    fingerMidpointXyErrorMm = Math.hypot(
      midpoint.x - renderedJaw.x,
      midpoint.y - renderedJaw.y
    ) * 1000;
    fingerTipAxialOffsetMm = Math.abs(midpoint.z - renderedJaw.z) * 1000;
    renderedFingertipLowZ = Math.min(leftFingerTip.z, rightFingerTip.z);
    renderedTableClearanceM = renderedFingertipLowZ;
    if (heightModel) {
      renderedFingerOverlapM = Math.max(
        0,
        Math.min(Number(heightModel.objectTopZ), renderedJaw.z)
          - Math.max(Number(heightModel.objectBottomZ), renderedFingertipLowZ)
      );
    }
  }

  const data = pickDiagnosticGroup.userData;
  data.objectCenter.position.copy(robotFrameToScene(objectCenter));
  data.requestedTcp.position.copy(robotFrameToScene(targetTcp));
  data.renderedJaw.position.copy(renderedJawScene);
  data.flange.position.copy(flangeScene);
  data.approach.position.copy(renderedJawScene);
  data.approach.setDirection(new THREE.Vector3(approach.x, approach.z, -approach.y).normalize());
  data.jaw.position.copy(renderedJawScene);
  data.jaw.setDirection(new THREE.Vector3(jaw.x, jaw.z, -jaw.y).normalize());
  pickDiagnosticGroup.visible = true;

  return {
    objectCenter,
    requestedTcp: clonePosition(targetTcp),
    requestedFlange: current.targetFlangePoseM ? clonePosition(current.targetFlangePoseM) : null,
    renderedJawCenter: renderedJaw,
    renderedFlangeOrigin: renderedFlange,
    renderedJawTargetErrorMm: targetErrorMm,
    renderedJawObjectXyErrorMm: objectXyErrorMm,
    renderedApproachTiltDeg: verticalErrorDeg,
    renderedApproachAxis: { x: approach.x, y: approach.y, z: approach.z },
    renderedJawAxis: { x: jaw.x, y: jaw.y, z: jaw.z },
    modelScale: Number(robot?.userData.modelScale || 0),
    jawHalfSpanM: GRIPPER_JAW_HALF_SPAN_OPEN_M,
    leftFingerTip,
    rightFingerTip,
    renderedFingerSpanMm,
    fingerMidpointErrorMm,
    fingerMidpointXyErrorMm,
    fingerTipAxialOffsetMm,
    fingerMidpoint,
    plannedHeightModel: heightModel ? { ...heightModel } : null,
    renderedFingertipLowZ,
    renderedFingerOverlapM,
    renderedTableClearanceM,
  };
}

export function startSimulation(plan) {
  const totalMs = plan.steps.reduce((sum, s) => sum + Number(s.durationMs || 800), 0);
  state.simulation = { plan, startedAt: performance.now(), totalMs, done: false, paused: false, pausedElapsed: 0 };
  state.simulationAngles = null;
  state.simulatedPartPositions.clear();
  state.gripperTargetOpen = GRIPPER_VISUAL_OPEN;
  renderPlanPath(plan);
  emit("simulation");
}

export function clearSimulation({ preservePath = false } = {}) {
  state.simulation = null;
  state.simulationAngles = null;
  state.simulatedPartPositions.clear();
  state.gripperTargetOpen = GRIPPER_VISUAL_OPEN;
  if (!preservePath) renderPlanPath(null);
  if (pickDiagnosticGroup) pickDiagnosticGroup.visible = false;
  renderEnvironment();
  emit("simulation");
}

export function getPickDiagnostics() {
  return state.simulation?.renderDiagnostics
    ? JSON.parse(JSON.stringify(state.simulation.renderDiagnostics))
    : null;
}

export function seekSimulationState(stateId, progress = 1) {
  const sim = state.simulation;
  if (!sim?.plan?.steps?.length) return false;
  const index = sim.plan.steps.findIndex((step) => step.stateId === stateId || step.name === stateId);
  if (index < 0) return false;
  const beforeMs = sim.plan.steps.slice(0, index)
    .reduce((sum, step) => sum + Number(step.durationMs || 800), 0);
  const durationMs = Number(sim.plan.steps[index].durationMs || 800);
  sim.startedAt = performance.now() - beforeMs - clamp(Number(progress), 0, 1) * durationMs;
  sim.pausedElapsed = beforeMs + clamp(Number(progress), 0, 1) * durationMs;
  sim.done = false;
  return true;
}

export function seekSimulationSource(sourceStepId, progress = 0) {
  const sim = state.simulation;
  if (!sim?.plan?.steps?.length) return false;
  const index = sim.plan.steps.findIndex((step) =>
    step.sourceStepId === sourceStepId || (step.sourceStepIds || []).includes(sourceStepId)
  );
  if (index < 0) return false;
  return seekSimulationState(sim.plan.steps[index].stateId, progress);
}

export function pauseSimulation() {
  const sim = state.simulation;
  if (!sim || sim.paused) return false;
  sim.pausedElapsed = Math.max(0, performance.now() - sim.startedAt);
  sim.paused = true;
  emit("simulation");
  return true;
}

export function resumeSimulation() {
  const sim = state.simulation;
  if (!sim) return false;
  sim.startedAt = performance.now() - Number(sim.pausedElapsed || 0);
  sim.paused = false;
  sim.done = false;
  emit("simulation");
  return true;
}

function stepTimeline(step, previousFinal) {
  const trajectory = (step.trajectory || []).map((p) => p.angles);
  if (trajectory.length) return [previousFinal, ...trajectory];
  if (Array.isArray(step.previewAngles) && step.previewAngles.length === 6) {
    return [previousFinal, step.previewAngles];
  }
  const target = step.jointAngles?.angles;
  return target ? [previousFinal, target] : [previousFinal, previousFinal];
}

function finalAnglesOf(step, previousFinal) {
  const trajectory = step.trajectory || [];
  if (trajectory.length) return trajectory[trajectory.length - 1].angles;
  if (Array.isArray(step.previewAngles) && step.previewAngles.length === 6) return step.previewAngles;
  return step.jointAngles?.angles || previousFinal;
}

function stepPose(step) {
  return step?.targetTcpPoseM || step?.targetPoseM || step?.pose || null;
}

function interpolatePose(a, b, t) {
  if (!a && !b) return null;
  if (!a) return b;
  if (!b) return a;
  const clamped = Math.max(0, Math.min(1, Number(t) || 0));
  return {
    x: Number(a.x || 0) + (Number(b.x || 0) - Number(a.x || 0)) * clamped,
    y: Number(a.y || 0) + (Number(b.y || 0) - Number(a.y || 0)) * clamped,
    z: Number(a.z || 0) + (Number(b.z || 0) - Number(a.z || 0)) * clamped,
  };
}

function plannedPoseAt(steps, currentIndex, progress) {
  const current = stepPose(steps[currentIndex]);
  if (!current) return null;
  let previous = null;
  for (let i = currentIndex - 1; i >= 0; i -= 1) {
    previous = stepPose(steps[i]);
    if (previous) break;
  }
  return interpolatePose(previous, current, progress);
}

function visualAnglesForCoordinatePose(pose, previousAngles) {
  if (!pose) return previousAngles ? previousAngles.slice() : null;

  const x = Number(pose.x || 0);
  const y = Number(pose.y || 0);
  const z = Number(pose.z || 0);
  const previous = previousAngles || state.angles || [0, 0, 0, 0, 0, -45];

  // Visual-only approximation for coordinate-mode simulation. Physical runs use
  // firmware send_coords and then update the twin from returned joint feedback.
  const baseYaw = clamp(radToDeg(Math.atan2(y, x)), -165, 165);
  const radial = Math.max(0.045, Math.hypot(x, y) - 0.045);
  const shoulderHeight = 0.14;
  const upper = 0.112;
  const forearm = 0.118;
  const dz = z - shoulderHeight;
  const reach = clamp(Math.hypot(radial, dz), 0.045, upper + forearm - 0.004);
  const shoulderLine = Math.atan2(dz, radial);
  const shoulderBend = Math.acos(clamp(
    (upper * upper + reach * reach - forearm * forearm) / Math.max(0.0001, 2 * upper * reach),
    -1,
    1
  ));
  const elbowInside = Math.acos(clamp(
    (upper * upper + forearm * forearm - reach * reach) / (2 * upper * forearm),
    -1,
    1
  ));
  const shoulder = radToDeg(shoulderLine + shoulderBend) - 25;
  const elbow = -(180 - radToDeg(elbowInside));
  const wrist = clamp(-shoulder - elbow - 8, -135, 135);

  return [
    baseYaw,
    clamp(shoulder, -120, 120),
    clamp(elbow, -145, 145),
    wrist,
    Number(previous[4] || 0),
    Number(previous[5] ?? -45),
  ];
}

function updateSimulation() {
  const sim = state.simulation;
  if (!sim?.plan) return;
  const elapsed = sim.paused ? Number(sim.pausedElapsed || 0) : performance.now() - sim.startedAt;
  const steps = sim.plan.steps;

  let cursor = 0;
  let currentIndex = steps.length - 1;
  let progress = 1;
  for (let i = 0; i < steps.length; i += 1) {
    const next = cursor + Number(steps[i].durationMs || 800);
    if (elapsed <= next) {
      currentIndex = i;
      progress = (elapsed - cursor) / Math.max(1, next - cursor);
      break;
    }
    cursor = next;
  }
  const done = elapsed >= sim.totalMs;
  if (done) { currentIndex = steps.length - 1; progress = 1; }
  const coordinatePlan = sim.plan.mode === "coordinate_program";
  const coordinatePreviewOk = coordinatePlan && Boolean(sim.plan.coordinatePreview?.ok);
  const canAnimatePlannedMotion = !coordinatePlan || coordinatePreviewOk;

  // Reconstruct held/placed state deterministically from completed steps.
  let heldId = null;
  let heldGrasp = null;
  state.simulatedPartPositions.clear();
  if (canAnimatePlannedMotion) {
    for (let i = 0; i < currentIndex; i += 1) {
      const s = steps[i];
      if (s.attachObjectId && s.name === "auto_grip") {
        heldId = s.attachObjectId;
        heldGrasp = s.grasp || heldGrasp;
      }
      if (s.releaseObjectId) {
        heldId = null;
        heldGrasp = null;
        if (s.placedPosition) state.simulatedPartPositions.set(s.releaseObjectId, s.placedPosition);
      }
    }
  }
  const current = steps[currentIndex];
  sim.currentStep = current;
  sim.currentIndex = currentIndex;
  if (canAnimatePlannedMotion && current.name === "auto_grip" && current.attachObjectId && progress >= 0.75) {
    heldId = current.attachObjectId;
    heldGrasp = current.grasp || heldGrasp;
  }
  if (canAnimatePlannedMotion && current.releaseObjectId && progress >= 0.45) {
    heldId = null;
    if (current.placedPosition) {
      state.simulatedPartPositions.set(current.releaseObjectId, current.placedPosition);
    }
  }

  // Gripper visual target.
  if (canAnimatePlannedMotion && (current.gripper === "closed" || (current.name === "auto_grip" && progress >= 0.3))) {
    state.gripperTargetOpen = GRIPPER_VISUAL_CLOSED;
  } else {
    state.gripperTargetOpen = GRIPPER_VISUAL_OPEN;
  }

  // Joint angles from the trajectory timeline.
  const coordinatePose = plannedPoseAt(steps, currentIndex, progress);
  if (coordinatePlan && !coordinatePreviewOk) {
    state.simulationAngles = state.angles ? state.angles.slice() : null;
  } else {
    let previousFinal = state.angles;
    for (let i = 0; i < currentIndex; i += 1) previousFinal = finalAnglesOf(steps[i], previousFinal);
    const timeline = stepTimeline(current, previousFinal);
    const slot = progress * (timeline.length - 1);
    const idx = Math.min(timeline.length - 2, Math.floor(slot));
    const local = slot - idx;
    const angles = interpolateAngles(timeline[idx], timeline[idx + 1], local);
    if (angles) {
      state.simulationAngles = angles;
    }
  }

  // Held parts follow the actual rendered jaw center.  Following the requested
  // coordinate pose used to conceal joint/model mismatches by visually moving
  // the object independently of the gripper.
  if (heldId) {
    const tcp = sceneToRobotFrame(getGripperTcpScenePosition());
    const part = findPart(heldId);
    const halfZ = part ? Number(part.size?.z || 0.05) / 2 : 0.02;
    const objectCenter = heldGrasp?.objectCenter;
    const graspPoint = heldGrasp?.graspPoint;
    const offset = objectCenter && graspPoint ? {
      x: Number(objectCenter.x || 0) - Number(graspPoint.x || 0),
      y: Number(objectCenter.y || 0) - Number(graspPoint.y || 0),
      z: Number(objectCenter.z || 0) - Number(graspPoint.z || 0),
    } : { x: 0, y: 0, z: 0 };
    state.simulatedPartPositions.set(heldId, {
      x: clamp(tcp.x + offset.x, -SCENE_BOUND_METERS, SCENE_BOUND_METERS),
      y: clamp(tcp.y + offset.y, -SCENE_BOUND_METERS, SCENE_BOUND_METERS),
      z: Math.max(halfZ, tcp.z + offset.z),
    });
  }

  if (done && !sim.done) {
    sim.done = true;
    emit("simulationDone");
  }
  sim.currentTargetTcp = coordinatePose || current.targetTcpPoseM || current.targetPoseM || current.pose || null;
  emit("simulationTick", {
    stepId: current.stateId,
    sourceStepId: current.sourceStepId || (current.sourceStepIds || [])[0] || null,
    index: currentIndex, progress, done,
    diagnostics: sim.renderDiagnostics || null,
  });
}

// ---------------------------------------------------------------- scene

function makeAxisLabel(text, color) {
  const canvas = document.createElement("canvas");
  canvas.width = 96; canvas.height = 48;
  const context = canvas.getContext("2d");
  context.font = "700 28px Inter, sans-serif";
  context.textAlign = "center";
  context.textBaseline = "middle";
  context.fillStyle = color;
  context.fillText(text, 48, 24);
  const texture = new THREE.CanvasTexture(canvas);
  texture.colorSpace = THREE.SRGBColorSpace;
  const sprite = new THREE.Sprite(new THREE.SpriteMaterial({ map: texture, transparent: true }));
  sprite.scale.set(0.34, 0.17, 1);
  return sprite;
}

function makeCoordinateKey() {
  const key = new THREE.Group();
  key.position.set(-3.45, 0.08, 3.25);
  const origin = new THREE.Vector3(0, 0, 0);
  key.add(new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), origin, 0.7, 0xef4444, 0.18, 0.09));
  key.add(new THREE.ArrowHelper(new THREE.Vector3(0, 0, -1), origin, 0.7, 0x16a34a, 0.18, 0.09));
  key.add(new THREE.ArrowHelper(new THREE.Vector3(0, 1, 0), origin, 0.7, 0x2563eb, 0.18, 0.09));
  const xl = makeAxisLabel("+X", "#ef4444"); xl.position.set(0.88, 0, 0);
  const yl = makeAxisLabel("+Y", "#16a34a"); yl.position.set(0, 0, -0.88);
  const zl = makeAxisLabel("+Z", "#2563eb"); zl.position.set(0, 0.88, 0);
  key.add(xl, yl, zl);
  return key;
}

function resizeRenderer() {
  const rect = viewport.getBoundingClientRect();
  const width = Math.floor(rect.width);
  const height = Math.floor(rect.height);
  if (width < 2 || height < 2) return;
  renderer.setSize(width, height, false);
  camera.aspect = width / height;
  camera.updateProjectionMatrix();
}

function updateCamera() {
  const target = new THREE.Vector3(0, 1.7, 0);
  const x = Math.sin(cameraYaw) * Math.cos(cameraPitch) * cameraDistance;
  const z = Math.cos(cameraYaw) * Math.cos(cameraPitch) * cameraDistance;
  const y = Math.sin(cameraPitch) * cameraDistance + 1.2;
  camera.position.set(x, y, z);
  camera.lookAt(target);
}

function animate() {
  requestAnimationFrame(animate);
  // Choose the current state and target before applying angles so the visual
  // diagnostics refer to the same simulation frame.
  if (state.simulation) updateSimulation();
  const target = state.simulation ? (state.simulationAngles || state.angles) : state.angles;
  for (let i = 0; i < 6; i += 1) {
    state.renderAngles[i] += (target[i] - state.renderAngles[i]) * RENDER_SMOOTHING;
  }
  applyAnglesToRobotFrames(state.renderAngles);
  if (state.simulation) {
    const now = performance.now();
    if (now - lastPickDiagnosticAt >= 80) {
      state.simulation.renderDiagnostics = updatePickDiagnostics(
        state.simulation.currentStep,
        state.simulation.currentTargetTcp
      );
      lastPickDiagnosticAt = now;
    }
  } else if (pickDiagnosticGroup) {
    pickDiagnosticGroup.visible = false;
  }
  updateEnvironmentTransforms(false);
  applyGripperState();
  updateCamera();
  renderer.render(scene, camera);
}

export async function initViewport(element, onStatus) {
  viewport = element;
  scene = new THREE.Scene();
  scene.background = new THREE.Color(0xeef2f6);
  camera = new THREE.PerspectiveCamera(50, 1, 0.05, 100);
  renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;
  renderer.outputColorSpace = THREE.SRGBColorSpace;
  renderer.toneMapping = THREE.ACESFilmicToneMapping;
  viewport.appendChild(renderer.domElement);
  if (viewportResizeObserver) viewportResizeObserver.disconnect();
  if (typeof ResizeObserver !== "undefined") {
    viewportResizeObserver = new ResizeObserver(() => resizeRenderer());
    viewportResizeObserver.observe(viewport);
  }

  scene.add(new THREE.HemisphereLight(0xffffff, 0x9fb0bd, 2.25));
  const key = new THREE.DirectionalLight(0xffffff, 2.7);
  key.position.set(4.5, 6, 3.4);
  key.castShadow = true;
  key.shadow.mapSize.set(2048, 2048);
  key.shadow.camera.left = -5; key.shadow.camera.right = 5;
  key.shadow.camera.top = 5; key.shadow.camera.bottom = -5;
  scene.add(key);
  const rim = new THREE.DirectionalLight(0xb8e7ff, 1.25);
  rim.position.set(-4, 3.5, -4);
  scene.add(rim);

  const floor = new THREE.Mesh(
    new THREE.PlaneGeometry(GRID_SIZE_UNITS, GRID_SIZE_UNITS),
    new THREE.MeshStandardMaterial({ color: 0xe8edf2, roughness: 0.9 })
  );
  floor.rotation.x = -Math.PI / 2;
  floor.receiveShadow = true;
  scene.add(floor);
  const grid = new THREE.GridHelper(GRID_SIZE_UNITS, 32, 0xcbd5df, 0xdce3ea);
  grid.position.y = 0.002;
  scene.add(grid);
  scene.add(makeCoordinateKey());

  environmentGroup = new THREE.Group();
  pathGroup = new THREE.Group();
  scene.add(environmentGroup, pathGroup);

  viewport.addEventListener("pointerdown", (event) => {
    if (startObjectDrag(event)) return;
    isDragging = true;
    previousPointer = { x: event.clientX, y: event.clientY };
    viewport.setPointerCapture(event.pointerId);
  });
  viewport.addEventListener("pointermove", (event) => {
    if (objectDrag) { updateObjectDrag(event); return; }
    if (!isDragging || !previousPointer) return;
    cameraYaw -= (event.clientX - previousPointer.x) * 0.008;
    cameraPitch = clamp(cameraPitch + (event.clientY - previousPointer.y) * 0.006, -0.05, 1.2);
    previousPointer = { x: event.clientX, y: event.clientY };
  });
  viewport.addEventListener("pointerup", () => {
    if (objectDrag) { commitObjectDrag(); return; }
    isDragging = false;
    previousPointer = null;
  });
  viewport.addEventListener("pointercancel", () => {
    objectDrag = null; state.dragActive = false; isDragging = false; previousPointer = null;
  });
  viewport.addEventListener("wheel", (event) => {
    event.preventDefault();
    cameraDistance = clamp(cameraDistance + event.deltaY * 0.004, 4.2, 14.0);
  }, { passive: false });
  window.addEventListener("resize", resizeRenderer);

  on("scene", () => { renderEnvironment(); syncEndEffector(); });
  on("tagTracks", () => updateEnvironmentTransforms(true));
  on("selection", renderEnvironment);

  resizeRenderer();
  animate();

  onStatus?.("Loading robot model...");
  robot = await createOfficialMyCobot();
  scene.add(robot);
  applyAnglesToRobotFrames(state.renderAngles);
  // Read-only diagnostics plus a deterministic simulation seeker make visual
  // regression inspection possible without touching the physical robot.
  window.__myCobotPickDebug = {
    diagnostics: getPickDiagnostics,
    seek: seekSimulationState,
    modelScale: () => Number(robot?.userData.modelScale || 0),
    modelBounds: () => JSON.parse(JSON.stringify(robot?.userData.unscaledBounds || null)),
  };
  onStatus?.("Robot model loaded.");
}
