/* [[file:../../blender.org::*FABRIK (Forward and Backward Reaching Inverse Kinematics)][FABRIK (Forward and Backward Reaching Inverse Kinematics):2]] */
package fabrik


import "core:fmt"
import "core:math/linalg"

Bone :: struct {
  name : string,
  head : linalg.Vector3f32,
  tail : linalg.Vector3f32,
  length : f32,
}

Bones :: distinct [dynamic]Bone

Chain :: struct {
  tolerance : f32,
  target : linalg.Vector3f32,
  bones : Bones,
  origin : linalg.Vector3f32,
  total_length : f32,
}

chain_backward :: proc(c: ^Chain) {
  c.bones[len(c.bones)-1].tail = c.target
  for i:=len(c.bones)-2; i >= 0; i-=1 {
    r := c.bones[i].tail - c.bones[i+1].tail
    rn := linalg.normalize(r)
    c.bones[i].tail = c.bones[i+1].tail + rn * c.bones[i].length     
  }
  for i:=1; i<len(c.bones); i+=1 {
    c.bones[i].head = c.bones[i-1].tail
  }
  r := c.bones[0].head - c.bones[1].head
  rn := linalg.normalize(r)
  c.bones[0].head = c.bones[1].head + rn * c.bones[0].length
}

chain_forward :: proc(c: ^Chain) {
  c.bones[0].head = c.origin
  for i:=1; i < len(c.bones); i+=1 {
    r := c.bones[i].head - c.bones[i-1].head
    rn := linalg.normalize(r)
    c.bones[i].head = c.bones[i-1].head + rn * c.bones[i].length     
  }
  for i:=1; i<len(c.bones); i+=1 {
    c.bones[i-1].tail = c.bones[i].head
  }
  i := len(c.bones)-1
  r := c.bones[i].tail - c.bones[i-1].tail
  rn := linalg.normalize(r)
  c.bones[i].tail = c.bones[i-1].tail + rn * c.bones[i].length
}

chain_solve :: proc(c: ^Chain) {
  dist := linalg.length(c.bones[0].head - c.target)
  if dist > c.total_length {
    // target is out of reach
    r := c.target - c.bones[0].head
    rn := linalg.normalize(r)
    for i:=1; i<len(c.bones); i+=1 {
      c.bones[i].head = c.bones[i-1].head + rn * c.bones[i].length
      c.bones[i-1].tail = c.bones[i].head
    }
    i := len(c.bones)-1
    c.bones[i].tail = c.bones[i].head + rn * c.bones[i].length
  } else {
    // target is in reach
    bcnt := 0
    diff := linalg.length(c.bones[len(c.bones)-1].tail - c.target)
    for ; diff > c.tolerance; {
      chain_backward(c)
      chain_forward(c)
      diff = linalg.length(c.bones[len(c.bones)-1].tail - c.target)
      bcnt += 1
      if bcnt >= 10 do break // avoid freezing
    }
  }
}
/* FABRIK (Forward and Backward Reaching Inverse Kinematics):2 ends here */
