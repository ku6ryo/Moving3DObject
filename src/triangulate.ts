import { Vector2 } from "./Vector2"

/**
 * Calculate the angle between two vectors.
 */
function calcDiffAngle(v1: Vector2, v2: Vector2) {
  const sin = v1.normalize().cross(v2.normalize())
  const cos = v1.normalize().dot(v2.normalize())
  if (sin >= 0) {
    return Math.acos(cos)
  } else {
    return 2 * Math.PI - Math.acos(cos)
  }
}

function isSameSide(a: Vector2, b: Vector2, p1: Vector2, p2: Vector2) {
  const v1 = b.sub(a)
  const v2 = p1.sub(a)
  const v3 = p2.sub(a)
  const c1 = v1.cross(v2)
  const c2 = v1.cross(v3)
  return c1 * c2 >= 0
}

function isPointInTriangle(a: Vector2, b: Vector2, c: Vector2, p: Vector2) {
  return isSameSide(a, b, c, p) && isSameSide(b, c, a, p) && isSameSide(c, a, b, p)
}

export function triangulate(
  points: Vector2[],
): [number, number, number][] {
  const pointIndices = points.map((_, i) => i)
  const triangles: [number, number, number][] = []
  while (pointIndices.length > 2) {
    const { index: i, triangle } = pointIndices.reduce((prev, _, i) => {
      const iP = pointIndices[i - 1] ?? pointIndices[pointIndices.length - 1]
      const iC = pointIndices[i]
      const iN = pointIndices[i + 1] ?? pointIndices[0]
      const pP = points[iP]
      const pC = points[iC]
      const pN = points[iN]
      const vCP = pP.sub(pC)
      const vCN = pN.sub(pC)
      const angle = calcDiffAngle(vCN, vCP)
      const diff = Math.abs(angle - Math.PI / 3)
      if (angle < Math.PI && diff < prev.diff) {
        const inTriangle = pointIndices.some((index) => {
          if (index === iP || index === iC || index === iN) return false
          return isPointInTriangle(pP, pC, pN, points[index])
        })
        if (inTriangle) return prev
        return { diff, index: i, triangle: [iP, iC, iN] }
      } else {
        return prev
      }
    }, { diff: Infinity, index: null as number | null, triangle: null as null | number[] })
    if (i === null || !triangle) throw new Error("Failed to find the best index")
    pointIndices.splice(i, 1)
    triangles.push(triangle as [number, number, number])
  }
  return triangles
}