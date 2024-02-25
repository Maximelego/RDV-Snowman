//
// Created by maxou on 25/02/24.
//

#include <iostream>

#include <GL/glew.h>
#include <GLFW/glfw3.h>

int main() {
    // Initialiser GLFW
    if (!glfwInit()) {
        return EXIT_FAILURE;
    }

    // Créer une fenêtre et son contexte OpenGL
    GLFWwindow* window = glfwCreateWindow(1280, 720, "Beautiful Snowman", NULL, NULL);
    if (!window) {
        glfwTerminate();
        return EXIT_FAILURE;
    }

    // Rendre le contexte de la fenêtre courant
    glfwMakeContextCurrent(window);

    // Initialiser GLEW
    if (glewInit() != GLEW_OK) {
        return EXIT_FAILURE;
    }

    // Boucle principale
    while (!glfwWindowShouldClose(window)) {
        // Dessiner ici


        // Échanger les tampons
        glfwSwapBuffers(window);

        // Poll for and process events
        glfwPollEvents();
    }

    glfwTerminate();
    return EXIT_SUCCESS;
}