#include <SFML/Graphics.hpp>
#include <iostream>
#include <queue>
#include <stack>
#include <vector>
#include <cmath>
#include <algorithm>

//실행시 visual studio 2022 기준으로 source\repos\(프로젝트 이름)\(프로젝트 이름)\ 에 arial.ttf를 넣어야한다.

constexpr int ROWS = 20;
constexpr int COLS = 30;
constexpr int CELL_SIZE = 25;

enum CellType { EMPTY, WALL, START, END, PATH, VISITED, FRONTIER };
enum Algorithm { BFS, DFS, DIJKSTRA, ASTAR };

struct Cell {
    int row, col;
    CellType type;
    float dist; // dijkstra
    float fScore; // A*
    Cell* parent;

    Cell(int r = 0, int c = 0) : row(r), col(c), type(EMPTY), dist(INFINITY), fScore(INFINITY), parent(nullptr) {}
};

class Grid {
public:
    std::vector<std::vector<Cell>> cells;

    Grid() {
        cells.resize(ROWS, std::vector<Cell>(COLS));
        for (int r = 0; r < ROWS; r++)
            for (int c = 0; c < COLS; c++)
                cells[r][c] = Cell(r, c);
    }

    void generateRandomWalls(float wallProbability = 0.3f) {
        for (int r = 0; r < ROWS; ++r) {
            for (int c = 0; c < COLS; ++c) {
                if ((r == 0 && c == 0) || (r == ROWS - 1 && c == COLS - 1)) continue; // 시작/끝 제외
                if ((float)rand() / RAND_MAX < wallProbability)
                    cells[r][c].type = WALL;
                else
                    cells[r][c].type = EMPTY;
            }
        }
    }

    bool isValid(int r, int c) {
        return r >= 0 && r < ROWS && c >= 0 && c < COLS;
    }

    std::vector<Cell*> getNeighbors(Cell& cell) {
        std::vector<Cell*> neighbors;
        int dr[] = { -1,1,0,0 };
        int dc[] = { 0,0,-1,1 };
        for (int i = 0; i < 4; i++) {
            int nr = cell.row + dr[i];
            int nc = cell.col + dc[i];
            if (isValid(nr, nc) && cells[nr][nc].type != WALL)
                neighbors.push_back(&cells[nr][nc]);
        }
        return neighbors;
    }
};

class Button {
    sf::RectangleShape rect;
    sf::Text text;
public:
    Button(float x, float y, float width, float height, const sf::Font& font, const std::string& str) {
        rect.setPosition(x, y);
        rect.setSize({ width, height });
        rect.setFillColor(sf::Color(200, 200, 200));
        rect.setOutlineColor(sf::Color::Black);
        rect.setOutlineThickness(2);

        text.setFont(font);
        text.setString(str);
        text.setCharacterSize(18);
        text.setFillColor(sf::Color::Black);

        sf::FloatRect textBounds = text.getLocalBounds();
        text.setOrigin(textBounds.left + textBounds.width / 2.0f,
            textBounds.top + textBounds.height / 2.0f);
        text.setPosition(x + width / 2.0f, y + height / 2.0f);
    }

    void draw(sf::RenderWindow& window) {
        window.draw(rect);
        window.draw(text);
    }

    bool isClicked(sf::Vector2f mousePos) {
        return rect.getGlobalBounds().contains(mousePos);
    }
};

class SpeedSlider {
    sf::RectangleShape bar;
    sf::RectangleShape knob;
    float minDelay, maxDelay;
    float* delayPtr;

public:
    SpeedSlider(float x, float y, float width, float height, float minD, float maxD, float* delay) {
        bar.setPosition(x, y + height / 2.0f);
        bar.setSize({ width, 5 });
        bar.setFillColor(sf::Color(150, 150, 150));

        knob.setSize({ 15, height });
        knob.setFillColor(sf::Color(100, 100, 250));
        knob.setOrigin(knob.getSize().x / 2.0f, knob.getSize().y / 2.0f);

        minDelay = minD;
        maxDelay = maxD;
        delayPtr = delay;

        setDelay(*delayPtr);
    }

    void draw(sf::RenderWindow& window) {
        window.draw(bar);
        window.draw(knob);
    }

    void setDelay(float delay) {
        if (delay < minDelay) delay = minDelay;
        if (delay > maxDelay) delay = maxDelay;

        *delayPtr = delay;
        float posX = bar.getPosition().x + ((delay - minDelay) / (maxDelay - minDelay)) * bar.getSize().x;
        knob.setPosition(posX, bar.getPosition().y);
    }

    bool isKnobClicked(sf::Vector2f mousePos) {
        return knob.getGlobalBounds().contains(mousePos);
    }

    void moveKnob(sf::Vector2f mousePos) {
        float x = mousePos.x;
        if (x < bar.getPosition().x) x = bar.getPosition().x;
        if (x > bar.getPosition().x + bar.getSize().x) x = bar.getPosition().x + bar.getSize().x;
        knob.setPosition(x, knob.getPosition().y);

        float ratio = (x - bar.getPosition().x) / bar.getSize().x;
        *delayPtr = minDelay + ratio * (maxDelay - minDelay);
    }
};

class PathfindingVisualizer {
public:
    Grid grid;
    Cell* startCell;
    Cell* endCell;
    Algorithm algo;

    bool running = false;
    bool finished = false;

    // BFS
    std::queue<Cell*> bfsQueue;

    // DFS
    std::stack<Cell*> dfsStack;

    // Dijkstra
    std::vector<Cell*> unvisited;

    // A*
    std::vector<Cell*> openSet;

    PathfindingVisualizer() : algo(BFS) {
        startCell = &grid.cells[0][0];
        startCell->type = START;
        endCell = &grid.cells[ROWS - 1][COLS - 1];
        endCell->type = END;
    }

    void resetGrid() {
        for (auto& row : grid.cells) {
            for (auto& cell : row) {
                cell.type = EMPTY;
                cell.dist = INFINITY;
                cell.fScore = INFINITY;
                cell.parent = nullptr;
            }
        }

        // 랜덤 벽 생성
        grid.generateRandomWalls();

        // 시작/끝 셀 재설정
        startCell = &grid.cells[0][0];
        endCell = &grid.cells[ROWS - 1][COLS - 1];
        startCell->type = START;
        endCell->type = END;

        running = false;
        finished = false;

        while (!bfsQueue.empty()) bfsQueue.pop();
        while (!dfsStack.empty()) dfsStack.pop();
        unvisited.clear();
        openSet.clear();
    }


    void changeAlgorithm(Algorithm a) {
        if (running) return;
        algo = a;
        resetGrid();
    }

    void startAlgorithm() {
        if (running) return;
        running = true;
        finished = false;

        // 초기화 + 시작 상태 설정
        if (algo == BFS) {
            bfsQueue.push(startCell);
        }
        else if (algo == DFS) {
            dfsStack.push(startCell);
        }
        else if (algo == DIJKSTRA) {
            for (auto& row : grid.cells) {
                for (auto& cell : row) {
                    cell.dist = INFINITY;
                    cell.parent = nullptr;
                    if (cell.type != WALL)
                        unvisited.push_back(&cell);
                }
            }
            startCell->dist = 0;
        }
        else if (algo == ASTAR) {
            for (auto& row : grid.cells) {
                for (auto& cell : row) {
                    cell.dist = INFINITY;
                    cell.fScore = INFINITY;
                    cell.parent = nullptr;
                }
            }
            startCell->dist = 0;
            startCell->fScore = heuristic(startCell, endCell);
            openSet.push_back(startCell);
        }
    }

    static float heuristic(Cell* a, Cell* b) {
        return std::abs(a->row - b->row) + std::abs(a->col - b->col);
    }

    void stepAlgorithm() {
        if (!running || finished) return;

        if (algo == BFS) stepBFS();
        else if (algo == DFS) stepDFS();
        else if (algo == DIJKSTRA) stepDijkstra();
        else if (algo == ASTAR) stepAStar();
    }

    void stepBFS() {
        if (bfsQueue.empty()) {
            finished = true;
            return;
        }

        Cell* current = bfsQueue.front();
        bfsQueue.pop();

        if (current == endCell) {
            reconstructPath(current);
            finished = true;
            running = false;
            return;
        }

        for (Cell* neighbor : grid.getNeighbors(*current)) {
            if (neighbor->type == EMPTY || neighbor->type == END) {
                neighbor->parent = current;
                if (neighbor->type != END) neighbor->type = FRONTIER;
                bfsQueue.push(neighbor);
            }
        }
        if (current->type != START) current->type = VISITED;
    }

    void stepDFS() {
        if (dfsStack.empty()) {
            finished = true;
            return;
        }

        Cell* current = dfsStack.top();
        dfsStack.pop();

        if (current == endCell) {
            reconstructPath(current);
            finished = true;
            running = false;
            return;
        }

        for (Cell* neighbor : grid.getNeighbors(*current)) {
            if (neighbor->type == EMPTY || neighbor->type == END) {
                neighbor->parent = current;
                if (neighbor->type != END) neighbor->type = FRONTIER;
                dfsStack.push(neighbor);
            }
        }
        if (current->type != START) current->type = VISITED;
    }

    void stepDijkstra() {
        if (unvisited.empty()) {
            finished = true;
            return;
        }

        // 최소 거리 셀 찾기
        auto comp = [](Cell* a, Cell* b) { return a->dist < b->dist; };
        auto currentIt = std::min_element(unvisited.begin(), unvisited.end(), comp);
        if (currentIt == unvisited.end() || (*currentIt)->dist == INFINITY) {
            finished = true;
            running = false;
            return;
        }

        Cell* current = *currentIt;
        unvisited.erase(currentIt);

        if (current == endCell) {
            reconstructPath(current);
            finished = true;
            running = false;
            return;
        }

        for (Cell* neighbor : grid.getNeighbors(*current)) {
            float alt = current->dist + 1;
            if (alt < neighbor->dist) {
                neighbor->dist = alt;
                neighbor->parent = current;
                if (neighbor->type != END) neighbor->type = FRONTIER;
            }
        }

        if (current->type != START) current->type = VISITED;
    }

    void stepAStar() {
        if (openSet.empty()) {
            finished = true;
            running = false;
            return;
        }

        // fScore 가장 낮은 노드 찾기
        auto comp = [](Cell* a, Cell* b) { return a->fScore < b->fScore; };
        auto currentIt = std::min_element(openSet.begin(), openSet.end(), comp);
        Cell* current = *currentIt;

        if (current == endCell) {
            reconstructPath(current);
            finished = true;
            running = false;
            return;
        }

        openSet.erase(currentIt);
        if (current->type != START) current->type = VISITED;

        for (Cell* neighbor : grid.getNeighbors(*current)) {
            float tentative_gScore = current->dist + 1;
            if (tentative_gScore < neighbor->dist) {
                neighbor->parent = current;
                neighbor->dist = tentative_gScore;
                neighbor->fScore = tentative_gScore + heuristic(neighbor, endCell);
                if (std::find(openSet.begin(), openSet.end(), neighbor) == openSet.end()) {
                    if (neighbor->type != END) neighbor->type = FRONTIER;
                    openSet.push_back(neighbor);
                }
            }
        }
    }

    void reconstructPath(Cell* end) {
        Cell* cur = end->parent;
        while (cur && cur != startCell) {
            cur->type = PATH;
            cur = cur->parent;
        }
    }

    void handleMouseClick(int x, int y) {
        int r = y / CELL_SIZE;
        int c = x / CELL_SIZE;
        if (grid.isValid(r, c)) {
            Cell& clicked = grid.cells[r][c];
            if (clicked.type == EMPTY)
                clicked.type = WALL;
            else if (clicked.type == WALL)
                clicked.type = EMPTY;
        }
    }
};

void drawCell(sf::RenderWindow& window, Cell& cell) {
    sf::RectangleShape rect(sf::Vector2f(CELL_SIZE - 1, CELL_SIZE - 1));
    rect.setPosition(cell.col * CELL_SIZE, cell.row * CELL_SIZE);

    switch (cell.type) {
    case EMPTY: rect.setFillColor(sf::Color::White); break;
    case WALL: rect.setFillColor(sf::Color::Black); break;
    case START: rect.setFillColor(sf::Color::Green); break;
    case END: rect.setFillColor(sf::Color::Red); break;
    case PATH: rect.setFillColor(sf::Color::Yellow); break;
    case VISITED: rect.setFillColor(sf::Color(150, 150, 255)); break;
    case FRONTIER: rect.setFillColor(sf::Color(100, 100, 255)); break;
    }

    window.draw(rect);
}

int main() {
    sf::RenderWindow window(sf::VideoMode(COLS * CELL_SIZE + 200, ROWS * CELL_SIZE), "Pathfinding Visualizer with GUI");

    sf::Font font;
    if (!font.loadFromFile("arial.ttf")) {
        std::cerr << "Failed to load font\n";
        return -1;
    }
    PathfindingVisualizer visualizer;
    visualizer.resetGrid(); // 처음 실행 시 랜덤 벽 생성

    // UI 버튼
    Button bfsBtn(COLS * CELL_SIZE + 20, 20, 160, 40, font, "BFS");
    Button dfsBtn(COLS * CELL_SIZE + 20, 70, 160, 40, font, "DFS");
    Button dijkstraBtn(COLS * CELL_SIZE + 20, 120, 160, 40, font, "Dijkstra");
    Button astarBtn(COLS * CELL_SIZE + 20, 170, 160, 40, font, "A*");
    Button startBtn(COLS * CELL_SIZE + 20, 230, 160, 40, font, "Start");
    Button resetBtn(COLS * CELL_SIZE + 20, 280, 160, 40, font, "Reset");

    float delay = 0.1f; // delay seconds
    SpeedSlider speedSlider(COLS * CELL_SIZE + 20, 340, 160, 30, 0.01f, 0.5f, &delay);

    sf::Clock clock;
    bool draggingSlider = false;

    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed)
                window.close();

            else if (event.type == sf::Event::MouseButtonPressed) {
                sf::Vector2f mousePos(event.mouseButton.x, event.mouseButton.y);
                if (bfsBtn.isClicked(mousePos)) visualizer.changeAlgorithm(BFS);
                else if (dfsBtn.isClicked(mousePos)) visualizer.changeAlgorithm(DFS);
                else if (dijkstraBtn.isClicked(mousePos)) visualizer.changeAlgorithm(DIJKSTRA);
                else if (astarBtn.isClicked(mousePos)) visualizer.changeAlgorithm(ASTAR);
                else if (startBtn.isClicked(mousePos)) visualizer.startAlgorithm();
                else if (resetBtn.isClicked(mousePos)) visualizer.resetGrid();
                else if (speedSlider.isKnobClicked(mousePos)) draggingSlider = true;
                else if (!visualizer.running && mousePos.x < COLS * CELL_SIZE) {
                    visualizer.handleMouseClick(mousePos.x, mousePos.y);
                }
            }
            else if (event.type == sf::Event::MouseButtonReleased) {
                draggingSlider = false;
            }
            else if (event.type == sf::Event::MouseMoved) {
                if (draggingSlider) {
                    sf::Vector2f mousePos(event.mouseMove.x, event.mouseMove.y);
                    speedSlider.moveKnob(mousePos);
                }
            }
        }

        if (visualizer.running && clock.getElapsedTime().asSeconds() >= delay) {
            visualizer.stepAlgorithm();
            clock.restart();
        }

        window.clear(sf::Color::White);

        // Draw grid
        for (auto& row : visualizer.grid.cells)
            for (auto& cell : row)
                drawCell(window, cell);

        // Draw UI
        bfsBtn.draw(window);
        dfsBtn.draw(window);
        dijkstraBtn.draw(window);
        astarBtn.draw(window);
        startBtn.draw(window);
        resetBtn.draw(window);
        speedSlider.draw(window);

        // Draw current algo text
        sf::Text algoText("Algorithm: ", font, 18);
        algoText.setFillColor(sf::Color::Black);
        algoText.setPosition(COLS * CELL_SIZE + 20, 390);
        window.draw(algoText);

        std::string algoStr;
        switch (visualizer.algo) {
        case BFS: algoStr = "BFS"; break;
        case DFS: algoStr = "DFS"; break;
        case DIJKSTRA: algoStr = "Dijkstra"; break;
        case ASTAR: algoStr = "A*"; break;
        }
        sf::Text algoName(algoStr, font, 18);
        algoName.setFillColor(sf::Color::Blue);
        algoName.setPosition(COLS * CELL_SIZE + 120, 390);
        window.draw(algoName);

        window.display();
    }
    
    return 0;
}
