#version 430

layout (location = 0) in vec3 position;
layout (location = 1) in vec3 normal;
layout (location = 2) in vec2 tex_coord;

layout (binding = 0, std140) uniform Camera
{
  mat4 projection;
  mat4 view;
};

uniform mat4 model;

out vec3 frag_position;
out vec3 frag_normal;
out vec2 frag_tex_coord;

void main()
{
  gl_Position = projection * view * model * vec4(position, 1.f);
  vec4 frag_homo_position = model * vec4(position, 1.f);
  frag_position = frag_homo_position.xyz / frag_homo_position.w;
  frag_normal = mat3(transpose(inverse(model))) * normal;
  frag_tex_coord = tex_coord;
}
