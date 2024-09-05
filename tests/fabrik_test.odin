/* [[file:../../../blender.org::*FABRIK (Forward and Backward Reaching Inverse Kinematics)][FABRIK (Forward and Backward Reaching Inverse Kinematics):1]] */
package fabrik_test


import "core:fmt"
import "core:mem"
import "core:strings"
import "core:math/linalg"
import "vendor:raylib"
import fabrik "../"

Values :: struct {
  show_menu : bool,
  chain_segments : i32,
}

draw_menu :: proc(w, h: i32, c: ^fabrik.Chain, values: ^Values) {
  using raylib

  if !values.show_menu {
    values.show_menu = GuiButton(Rectangle{f32(w) - 40, 13, 18, 18}, "_")
  } else {
    panel := GuiPanel(Rectangle{f32(w) - 210, 10, 200, 430}, "")
    values.show_menu = !GuiButton(Rectangle{f32(w) - 40, 13, 18, 18}, "_")

    tmp_chain_segments := f32(values.chain_segments)
    GuiSlider(Rectangle{f32(w) - 175, 40, 150, 20}, "segs", "", &tmp_chain_segments, 3, 24)
    if i32(tmp_chain_segments) != values.chain_segments {
      change_chain(c, values)
    }
    values.chain_segments = i32(tmp_chain_segments)
    str := fmt.tprintf("%v", i32(tmp_chain_segments))
    cstr := strings.clone_to_cstring(str)
    GuiTextBox(Rectangle{f32(w) - 175, 40, 150, 20}, cstr, 10, false)
    delete(cstr)
  }
}

change_chain :: proc(c: ^fabrik.Chain, values: ^Values) {
  using fabrik
  
  clear(&c.bones)
  d : i32 = 20

  for i in 0..<values.chain_segments {
    b : Bone
    b.name = fmt.tprintf("bone%d", i)
    b.head = linalg.Vector3f32{f32(i*d) + c.origin.x, f32(i*d) + c.origin.y, f32(i*1)}
    b.tail = linalg.Vector3f32{f32(i*d+d)+c.origin.x, f32(i*d+d)+c.origin.y, f32(i*1+1)}
    b.length = linalg.length(b.tail - b.head)
    c.total_length += b.length
    append(&c.bones, b)
  }
}

main :: proc() {
  using raylib
  using fabrik

  ta := mem.Tracking_Allocator{};
  mem.tracking_allocator_init(&ta, context.allocator);
  context.allocator = mem.tracking_allocator(&ta);

  {
    w,h : i32
    WIDTH  :: 500
    HEIGHT :: 500

    SetConfigFlags(ConfigFlags{ConfigFlag.WINDOW_RESIZABLE})
    InitWindow(WIDTH, HEIGHT, "FABRIK testing")
    SetTargetFPS(60)

    c : Chain
    c.tolerance = 0.01
    c.target = linalg.Vector3f32{0,0,0}
    c.origin = linalg.Vector3f32{150,150,0}
    c.total_length = 0

    values : Values
    values.chain_segments = 10
    change_chain(&c, &values)
    defer delete(c.bones)

    radius : f32 = 3
    color := WHITE

    for !WindowShouldClose() {
      // Update ------------------------------
      w = GetScreenWidth()
      h = GetScreenHeight()

      e := GetMousePosition()
      c.target.x = e.x
      c.target.y = e.y
      chain_solve(&c)

      // Draw   ------------------------------
      BeginDrawing()
      ClearBackground(BLACK)

      for b, idx in c.bones {
        DrawCircleLines(cast(i32)b.head.x, cast(i32)b.head.y, radius, color)
        DrawLine(cast(i32)b.head.x, cast(i32)b.head.y, cast(i32)b.tail.x, cast(i32)b.tail.y, WHITE)
        if idx == len(c.bones)-1 {
          DrawCircleLines(cast(i32)b.tail.x, cast(i32)b.tail.y, radius, color)
        }
      }

      draw_menu(w, h, &c, &values)

      EndDrawing()
    }
    CloseWindow()
  }
  if len(ta.allocation_map) > 0 {
    for _, v in ta.allocation_map {
      fmt.printf("Leaked %v bytes @ %v\n", v.size, v.location)
    }
  }
  if len(ta.bad_free_array) > 0 {
    fmt.println("Bad frees:")
    for v in ta.bad_free_array {
      fmt.println(v)
    }
  }
}
/* FABRIK (Forward and Backward Reaching Inverse Kinematics):1 ends here */
