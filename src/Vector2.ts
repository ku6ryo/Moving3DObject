export class Vector2 {
  x: number
  y: number

  constructor (x: number, y: number) {
    this.x = x
    this.y = y
  }

  add (other: Vector2) {
    return new Vector2(this.x + other.x, this.y + other.y)
  }

  sub (other: Vector2) {
    return new Vector2(this.x - other.x, this.y - other.y)
  }

  multiply (scalar: number) {
    return new Vector2(this.x * scalar, this.y * scalar)
  }

  divide (scalar: number) {
    return new Vector2(this.x / scalar, this.y / scalar)
  }

  dot (other: Vector2) {
    return this.x * other.x + this.y * other.y
  }

  cross (other: Vector2) {
    return this.x * other.y - this.y * other.x
  }

  lengthSquared () {
    return this.x * this.x + this.y * this.y
  }

  length () {
    return Math.sqrt(this.lengthSquared())
  }

  normalize () {
    const length = this.length()
    return new Vector2(this.x / length, this.y / length)
  }

}