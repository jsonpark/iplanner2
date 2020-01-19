#version 430

in vec2 vertex_tex_coord;

uniform sampler2D tex;
uniform usampler2D depth;

out vec4 out_color;

void main()
{
  // Color
  //out_color = vec4(texture(tex, vertex_tex_coord).rgb, 1.f);

  // Depth
  uint du = texture(depth, vertex_tex_coord).r;
  float d = (float(du) / 1000.f) / 2.f;
  out_color = vec4(vec3(d), 1.f);
}
