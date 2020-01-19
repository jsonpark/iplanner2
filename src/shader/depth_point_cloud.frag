#version 430 core

in vec4 frag_position;

out uint out_depth;

void main()
{
  out_depth = uint(-frag_position.z * 1000.f);
}
