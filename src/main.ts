import { NodeIO } from "@gltf-transform/core"
import { Vector2 } from "./Vector2"
import { createThickAnimatedObject } from "./createThickAnimatedObject"

function main() {
  const points = [] as Vector2[]
  const divisions = 7
  for (let i = 0; i < divisions; i++) {
    const theta = 2 * Math.PI / divisions * i + Math.PI / 2
    points.push(new Vector2(Math.cos(theta), Math.sin(theta)))
  }
  const doc = createThickAnimatedObject(points)
  const io = new NodeIO()
  io.write("thickObj.glb", doc)
}

main()