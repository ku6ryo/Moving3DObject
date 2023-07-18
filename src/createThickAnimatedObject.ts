import {
  Document,
  Accessor,
} from "@gltf-transform/core"
import { mat4, vec3, quat } from "gl-matrix"
import { triangulate } from "./triangulate"
import { Vector2 } from "./Vector2"

export function createThickAnimatedObject(points: Vector2[], thickness: number) {
  const doc = new Document()
  const buffer = doc.createBuffer("dataBuffer")
  const skin = doc.createSkin("skin")
  const originJointPos = vec3.fromValues(0, 0, 0)
  const originJoint = doc.createNode(`origin-joint`)
    .setTranslation([0, 0, 0])
    .setScale([1, 1, 1])
    .setRotation([0, 0, 0, 1])
  skin.addJoint(originJoint)
  const jointAngles = [Math.PI / 2, - Math.PI / 3, Math.PI * 4 / 3]
  const centerJointPositions = jointAngles.map(angle => {
    return vec3.fromValues(Math.cos(angle) * 0.5, Math.sin(angle) * 0.5, 0)
  })
  const edgeJointPositions = jointAngles.map(angle => {
    return vec3.fromValues(Math.cos(angle) * 0.5, Math.sin(angle) * 0.5, 0)
  })
  const edgeJoints = edgeJointPositions.map((pos, i) => {
    return doc.createNode(`joint-edge-${i}`)
      .setTranslation([pos[0], pos[1], pos[2]])
      .setScale([1, 1, 1])
      .setRotation([0, 0, 0, 1])
  })
  const centerJoints = centerJointPositions.map((pos, i) => {
    return doc.createNode(`joint-center-${i}`)
      .setTranslation([pos[0], pos[1], pos[2]])
      .setScale([1, 1, 1])
      .setRotation([0, 0, 0, 1])
  })
  centerJoints.forEach((joint, i) => {
    skin.addJoint(joint)
    originJoint.addChild(joint)
  })
  edgeJoints.forEach((joint, i) => {
    skin.addJoint(joint)
    centerJoints[i].addChild(joint)
  })

  const inverseBindMatrices = [] as mat4[]
  const jointCenterMatrix = mat4.fromTranslation(mat4.create(), originJointPos)
  mat4.invert(jointCenterMatrix, jointCenterMatrix)
  inverseBindMatrices.push(jointCenterMatrix)

  for (const pos of centerJointPositions) {
    const mat = mat4.fromTranslation(mat4.create(), pos)
    mat4.invert(mat, mat)
    inverseBindMatrices.push(mat)
  }
  edgeJointPositions.forEach((pos, i) => {
    const cj = centerJointPositions[i]
    const mat = mat4.fromTranslation(mat4.create(), vec3.add(vec3.create(), pos, cj))
    mat4.invert(mat, mat)
    inverseBindMatrices.push(mat)
  })
  const flatInverseBindMatrices = [] as number[]
  for (const m of inverseBindMatrices) {
    for (let i = 0; i < 16; i++) {
      flatInverseBindMatrices.push(m[i])
    }
  }

  const inverseBindMatricesAccessor = doc
    .createAccessor("inverseBindMatrices")
    .setArray(new Float32Array(flatInverseBindMatrices))
    .setType(Accessor.Type.MAT4)
    .setBuffer(buffer)

  const triangles = triangulate(points)

  const numPoints = points.length
  const frontPoints = points.map((v) => [v.x, v.y, -thickness / 2])
  const frontIndices = Array.from({ length: numPoints }).map((_, i) => i)
  const frontTris = triangles.map(t => [t[0], t[2], t[1]])
  const backPoints = points.map((v) => [v.x, v.y, thickness / 2])
  const backIndices = Array.from({ length: numPoints }).map((_, i) => i + numPoints)
  const backTris = triangles.map(t => [t[0] + numPoints, t[1] + numPoints, t[2] + numPoints])
  const edgeDivisions = 3
  const edgePoints = [] as number[][][]
  const edgeIndices = [frontIndices] as number[][]
  for (let i = 0; i < edgeDivisions - 1; i++) {
    const simplePoints = points.map((c, j) => {
      const p = j === 0 ? points[points.length - 1] : points[j - 1]
      const n = points[(j + 1) % points.length]
      const vCP = p.sub(c).normalize()
      const vCN = n.sub(c).normalize()
      const sin = vCP.cross(vCN)
      const o = vCP.add(vCN).normalize().multiply(sin / Math.abs(sin))
      const phase = Math.PI / edgeDivisions * (i + 1)
      const { x, y } = calcOval(thickness / 2, thickness / 4, phase)
      const ot = o.multiply(y)
      return [c.x + ot.x, c.y + ot.y, -x]
    })
    const indices = Array.from({ length: numPoints }).map((_, j) => j + numPoints * 2 + i * numPoints)
    edgePoints.push(simplePoints)
    edgeIndices.push(indices)
  }
  edgeIndices.push(backIndices)
  const edgeTris = [] as number[][]
  for (let i = 0; i < edgeDivisions; i++) {
    const triangles = fillGapBtwTwoPathes(edgeIndices[i], edgeIndices[i + 1])
    edgeTris.push(...triangles)
  }
  const allPoints = frontPoints.concat(backPoints).concat(edgePoints.flat())
  const allIndices = frontTris.concat(backTris).concat(edgeTris)

  const vertsAccessor = doc.createAccessor("vertsAccessor")
    .setArray(new Float32Array(allPoints.flat()))
    .setType(Accessor.Type.VEC3)
    .setBuffer(buffer)

  const indicesAccessor = doc.createAccessor("indicesAccessor")
    .setArray(new Uint32Array(allIndices.flat()))
    .setType(Accessor.Type.SCALAR)
    .setBuffer(buffer)
  
  skin.setSkeleton(originJoint)

  const jointAssignments = [] as [number, number, number, number][]
  const weights = [] as [number, number, number, number][]

  allPoints.forEach((p) => {
    const min = edgeJointPositions.reduce((result, jP ,i) => {
      const len = vec3.distance(p as [number, number, number], jP)
      if (len < result.len) {
        return {
          len,
          i,
        }
      } else {
        return result
      }
    }, {
      len: Infinity,
      i: -1,
    })
    const jointAssignment = [0, 0, 0, 0]
    jointAssignment[0] = min.i + 1 + jointAngles.length
    jointAssignments.push(jointAssignment as [number, number, number, number])
    weights.push([1, 0, 0, 0])
  })

  const jointsAccessor = doc
    .createAccessor("jointAssignments")
    // Joint assignments must be stored as unsigned bytes or unsingned shorts, Uint8Array or Uint16Array.
    .setArray(new Uint8Array(jointAssignments.flat()))
    .setType(Accessor.Type.VEC4)
    .setBuffer(buffer)

  const weightsAccessor = doc
    .createAccessor("weights")
    .setArray(new Float32Array(weights.flat()))
    .setType(Accessor.Type.VEC4)
    .setBuffer(buffer)

  const primitive = doc.createPrimitive()
    .setAttribute("POSITION", vertsAccessor)
    .setIndices(indicesAccessor)
    .setAttribute("JOINTS_0", jointsAccessor)
    .setAttribute("WEIGHTS_0", weightsAccessor)
  const mesh = doc.createMesh("mesh").addPrimitive(primitive)
  const polygon = doc.createNode("polygon").setMesh(mesh)
  polygon.setSkin(skin)

  skin.setInverseBindMatrices(inverseBindMatricesAccessor)

  const times = doc.createAccessor("times")
    .setArray(new Float32Array([0, 1, 2]))
    .setType(Accessor.Type.SCALAR)
    .setBuffer(buffer)

  const animation = doc.createAnimation("animation")
  const deltaAngle = Math.PI / 12
  jointAngles.forEach((_, i) => {
    const rotations = [] as quat[]
    const sign = i % 2 === 0 ? 1 : -1
    const angles = [
      deltaAngle * sign,
      - deltaAngle * sign,
      deltaAngle * sign,
    ]
    for (const a of angles) {
      rotations.push(quat.fromEuler(quat.create(), 0, 0, a / Math.PI * 180))
    }
    const accessor = doc
      .createAccessor(`joint-anim-rot-${i}`)
      .setArray(new Float32Array(rotations.map(m => [m[0], m[1], m[2], m[3]]).flat()))
      .setType(Accessor.Type.VEC4)
      .setBuffer(buffer)
    const sampler = doc.createAnimationSampler(`joint-anim-sampler-${i}`)
      .setInput(times)
      .setOutput(accessor)
      .setInterpolation("LINEAR")
    const channel = doc.createAnimationChannel(`joint-anim-channel-${i}`)
      .setTargetNode(centerJoints[i])
      .setTargetPath("rotation")
      .setSampler(sampler)
    
    animation.addSampler(sampler)
    animation.addChannel(channel)
  })

  const container = doc.createNode("container")
  container.addChild(polygon)
  container.addChild(originJoint)
  doc.createScene().addChild(container)

  return doc
}

function calcOval(a: number, b: number, phase: number) {
  return {
    x: a * Math.cos(phase),
    y: b * Math.sin(phase),
  }
}

function fillGapBtwTwoPathes(path1: number[], path2: number[]) {
  if (path1.length !== path2.length) {
    throw new Error("Pathes must have the same length")
  }
  const triangles: [number, number, number][] = []
  for (let i = 0; i < path1.length; i++) {
    const p11 = path1[i]
    const p12 = path1[(i + 1) % path1.length]
    const p21 = path2[i]
    const p22 = path2[(i + 1) % path1.length]
    triangles.push([p11, p12, p22])
    triangles.push([p11, p22, p21])
  }
  return triangles
}