#pragma once
#include<iostream>
#include <SFML/Graphics.hpp>
#include <vector>
#include <fstream>
#include <sstream>
#include <math.h>
using namespace std;
class Simulator {
    private:
        vector<sf::Vector2f> waypoints;
        sf::RenderTexture waypointTexture;
        sf::Sprite waypointSprite;
        sf::RenderWindow window;  //Window that can serve as a target for 2D drawing
        void load_waypoints(const string&);
        float scale ;
        float offset_x ;
        float offset_y ;
        sf::View view;
        float zoom_factor = 1.0f;
        sf::Vector2f lastMousePos;
        bool isDragging = false;

    public:
        Simulator(int&, int&, std::string& );
        void run(const double&, const double&);
};