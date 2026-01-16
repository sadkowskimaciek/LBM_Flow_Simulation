#include <SFML/Graphics.hpp>
#include <vector>
#include <cmath>
#include <iostream>
#include <algorithm>

// === STAŁE SYMULACJI ===
const int WIDTH = 400;       // Szerokość domeny
const int HEIGHT = 150;      // Wysokość domeny
const int Q = 9;             // Model D2Q9 [cite: 17]
const double TAU = 1.0;      // Czas relaksacji [cite: 49]

// Wagi dla D2Q9 [cite: 42]
const double w[Q] = {
    4.0/9.0,
    1.0/9.0, 1.0/9.0, 1.0/9.0, 1.0/9.0,
    1.0/36.0, 1.0/36.0, 1.0/36.0, 1.0/36.0
};

// Wektory kierunkowe e_i = [cx, cy] [cite: 20]
// Indeksy: 0:C, 1:E, 2:N, 3:W, 4:S, 5:NE, 6:NW, 7:SW, 8:SE
const int cx[Q] = { 0,  1,  0, -1,  0,  1, -1, -1,  1};
const int cy[Q] = { 0,  0,  1,  0, -1,  1,  1, -1, -1};
// Tablica kierunków przeciwnych (do Bounce-Back) [cite: 58]
const int opposite[Q] = { 0, 3, 4, 1, 2, 7, 8, 5, 6 };

struct Node {
    double f[Q];     // Funkcja rozkładu (bieżąca)
    double f_new[Q]; // Funkcja rozkładu (po streamingu)
    double rho;      // Gęstość
    double ux;       // Prędkość X
    double uy;       // Prędkość Y
    bool isWall;     // Czy węzeł jest ścianą (przeszkodą)
};

// Siatka symulacyjna
std::vector<Node> grid(WIDTH * HEIGHT);

// === FUNKCJE POMOCNICZE ===

// Dostęp do siatki 2D przez indeks 1D
inline Node& getNode(int x, int y) {
    return grid[y * WIDTH + x];
}

// Inicjalizacja warunków początkowych [cite: 65, 66, 67]
void init() {
    for (int y = 0; y < HEIGHT; ++y) {
        for (int x = 0; x < WIDTH; ++x) {
            Node& n = getNode(x, y);

            n.isWall = false;
            if (x == WIDTH / 2) {

                if (y < HEIGHT * 0.4 || y > HEIGHT * 0.6) {
                    n.isWall = true;
                }
            }
            if (y == 0 || y == HEIGHT - 1) n.isWall = true;

            if (x < WIDTH / 2) n.rho = 1.00;
            else               n.rho = 0.98;

            n.ux = 0.0;
            n.uy = 0.0;
            double u_sq = 0.0;
            for (int i = 0; i < Q; ++i) {

                n.f[i] = w[i] * n.rho;
            }
        }
    }
}

// Krok symulacji LBM
void step() {
    for (int y = 0; y < HEIGHT; ++y) {
        for (int x = 0; x < WIDTH; ++x) {
            Node& n = getNode(x, y);

            if (n.isWall) continue;

            double sum_f = 0.0;
            double sum_f_cx = 0.0;
            double sum_f_cy = 0.0;
            for (int i = 0; i < Q; ++i) {
                sum_f += n.f[i];
                sum_f_cx += n.f[i] * cx[i];
                sum_f_cy += n.f[i] * cy[i];
            }
            n.rho = sum_f;
            if (n.rho > 0) {
                n.ux = sum_f_cx / n.rho; // [cite: 28]
                n.uy = sum_f_cy / n.rho;
            } else {
                n.ux = n.uy = 0;
            }

            double usq = n.ux * n.ux + n.uy * n.uy; // u^2

            for (int i = 0; i < Q; ++i) {
                double vu = cx[i] * n.ux + cy[i] * n.uy; // c_i * u

                double term1 = 3.0 * vu;
                double term2 = 4.5 * vu * vu;
                double term3 = 1.5 * usq;

                double f_eq = w[i] * n.rho * (1.0 + term1 + term2 - term3);

                // Kolizja BGK Eq (4) [cite: 48]
                n.f[i] = n.f[i] - (1.0 / TAU) * (n.f[i] - f_eq);
            }
        }
    }

    for (int y = 0; y < HEIGHT; ++y) {
        for (int x = 0; x < WIDTH; ++x) {
            Node& source = getNode(x, y);

            // Dla każdego kierunku i
            for (int i = 0; i < Q; ++i) {
                int nextX = x + cx[i];
                int nextY = y + cy[i];

                if (nextX < 0 || nextX >= WIDTH || nextY < 0 || nextY >= HEIGHT) {
                    // Poza domeną - Bounce-back
                    getNode(x, y).f_new[opposite[i]] = source.f[i];
                } else {
                    Node& target = getNode(nextX, nextY);

                    if (target.isWall) {
                        // Odbicie od przeszkody (Bounce-back) [cite: 57]
                        getNode(x, y).f_new[opposite[i]] = source.f[i];
                    } else {
                        // Normalny streaming
                        target.f_new[i] = source.f[i];
                    }
                }
            }
        }
    }

    // Aktualizacja stanu: f = f_new
    for (int i = 0; i < grid.size(); ++i) {
        if(!grid[i].isWall) {
            for(int k=0; k<Q; k++) grid[i].f[k] = grid[i].f_new[k];
        }
    }
}

// === WIZUALIZACJA ===

// Funkcja mapująca prędkość na kolor
sf::Color getColor(double val, double maxVal) {
    if (std::isnan(val)) return sf::Color::Black;

    double intensity = std::abs(val) / maxVal * 255.0 * 5.0;
    if (intensity > 255) intensity = 255;

    sf::Uint8 c = static_cast<sf::Uint8>(intensity);

    // [cite: 69] (czerwony dla dodatnich, niebieski dla ujemnych)
    if (val > 0) return sf::Color(c, 0, 0);
    if (val < 0) return sf::Color(0, 0, c);
    return sf::Color::Black;
}

int main() {
    sf::RenderWindow window(sf::VideoMode(WIDTH, HEIGHT * 2), "LBM Fluid Flow Task 3 - D2Q9 [Press R to Reset]");
    window.setFramerateLimit(60);

    // Inicjalizacja początkowa
    init();

    sf::Image image;
    image.create(WIDTH, HEIGHT * 2, sf::Color::Black);
    sf::Texture texture;
    sf::Sprite sprite;

    sf::Font font;
    if(!font.loadFromFile("arial.ttf")) {
        // Brak czcionki nie jest błędem krytycznym
    }
    sf::Text textUx("Ux (Horizontal)", font, 14);
    textUx.setPosition(5, 5);
    textUx.setFillColor(sf::Color::White);

    sf::Text textUy("Uy (Vertical)", font, 14);
    textUy.setPosition(5, HEIGHT + 5);
    textUy.setFillColor(sf::Color::White);

    int iter = 0;
    bool paused = false;

    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed)
                window.close();

            // --- NOWA SEKCJA: RESETOWANIE ---
            if (event.type == sf::Event::KeyPressed) {
                if (event.key.code == sf::Keyboard::R) {
                    std::cout << "Resetting simulation..." << std::endl;
                    init();    // Przywróć stan początkowy (gęstości i prędkości)
                    iter = 0;  // Wyzeruj licznik
                }
            }
            if (event.type == sf::Event::KeyPressed) {
                if (event.key.code == sf::Keyboard::Space) {
                    std::cout << "Stopping simulation..." << std::endl;
                    paused = !paused;
                }
            }
            // --------------------------------
        }

        if (!paused) {
            step();
            iter++;
        }

        double maxU = 0.15;

        for (int y = 0; y < HEIGHT; ++y) {
            for (int x = 0; x < WIDTH; ++x) {
                Node& n = getNode(x, y);

                if (n.isWall) {
                    image.setPixel(x, y, sf::Color(100, 100, 100));
                    image.setPixel(x, y + HEIGHT, sf::Color(100, 100, 100));
                } else {
                    image.setPixel(x, y, getColor(n.ux, maxU));
                    image.setPixel(x, y + HEIGHT, getColor(n.uy, maxU));
                }
            }
        }

        for(int x=0; x<WIDTH; x++) image.setPixel(x, HEIGHT, sf::Color::White);

        texture.loadFromImage(image);
        sprite.setTexture(texture);

        window.clear();
        window.draw(sprite);
        if (font.getInfo().family != "") {
             window.draw(textUx);
             window.draw(textUy);
        }
        window.display();

        if(iter % 10 == 0) std::cout << "Iteration: " << iter << std::endl;
    }

    return 0;
}