#version 430

layout (points) in;
layout (triangle_strip, max_vertices = 4) out;

in Vertex
{
  vec4 position;
  float width;
  float height;
} frag[];

out vec4 frag_position;

void main()
{
  frag_position = frag[0].position;
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
