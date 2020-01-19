#version 430

layout (location = 0) in vec3 position;

layout (binding = 0, std140) uniform Camera
{
  uniform mat4 projection;
  uniform mat4 view;
};

uniform mat4 model;

uniform vec4 width_height_near_far;

out Vertex
{
  vec4 position;
  float width;
  float height;
} frag;

void main()
{
  vec4 vm = view * model * vec4(position, 1.f);
  gl_Position = projection * vm;
  
  frag.position = vec4(view * model * vec4(position, 1.f));
  frag.width = (-vm.z * 2.f / width_height_near_far[0]) * 2.0;
  frag.height = (-vm.z * 2.f / width_height_near_far[1]) * 2.0;
}
