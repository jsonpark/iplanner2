#version 430

in vec2 vertex_tex_coord;

uniform usampler2D depth;

out vec4 out_color;

void main()
{
  uint du = texture(depth, vertex_tex_coord).r;

  if (du == 0)
    du = 8000;

  float d = (float(du) / 1000.f) / 8.f; // 8 meters maps to RGB (1, 1, 1)
  out_color = vec4(vec3(d), 1.f);
}
