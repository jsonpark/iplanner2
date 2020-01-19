#version 430

in vec4 vertex_position;

out uint out_depth;

void main()
{
  out_depth = uint(-vertex_position.z * 1000.f);
}
