#version 330 core
out vec4 FragColor;
  
in vec3 fColor;

void main() {
    FragColor = vec4(1.-fColor, 1.0);
}