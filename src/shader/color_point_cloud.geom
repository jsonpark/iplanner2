#version 430

layout (points) in;
layout (triangle_strip, max_vertices = 4) out;

in Vertex
{
  vec3 color;
  float width;
  float height;
} frag[];

out vec3 frag_color;

void main()
{
  frag_color = frag[0].color;
  float width = frag[0].width;
  float height = frag[0].height;

  gl_Position = gl_in[0].gl_Position + vec4(-width, -height, 0.0f, 0.0f); 
  EmitVertex();
  gl_Position = gl_in[0].gl_Position + vec4(width, -height, 0.0f, 0.0f);
  EmitVertex();
  gl_Position = gl_in[0].gl_Position + vec4(-width, height, 0.0f, 0.0f);
  EmitVertex();
  gl_Position = gl_in[0].gl_Position + vec4(width, height, 0.0f, 0.0f);
  EmitVertex();
  EndPrimitive();
}
