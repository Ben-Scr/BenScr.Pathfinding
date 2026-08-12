#include <raylib.h>
#include <cstddef>
#include <array>
#include <limits>
#include <random>
#include <cmath>
#include <queue>

class AStarAlgorithm {
public:
    static constexpr int NODE_ROW_AMOUNT = 20 * 1;
    static constexpr int NODE_PX_SIZE = 50 / 1;
    static constexpr int OBSTACLES_AMOUNT = NODE_ROW_AMOUNT * 5;

    AStarAlgorithm() {
        for (int y{}; y < NODE_ROW_AMOUNT; y++) {
            for (int x{}; x < NODE_ROW_AMOUNT; x++) {
                m_grid[y][x] = Node{ x, y };
            }
        }

        m_end = &m_grid[NODE_ROW_AMOUNT-1][NODE_ROW_AMOUNT-1];

        m_start = &m_grid[0][0];
        m_start->GCost = 0.0f;
        m_start->HCost = heuristic(m_start, m_end);
        m_start->FCost = m_start->GCost + m_start->HCost;
        m_start->IsOpen = true;
        m_opened.push(OpenNode(m_start));

        std::mt19937 generator{ std::random_device()()};
        std::uniform_int_distribution<int> range(0, NODE_ROW_AMOUNT - 1);

        for (int i{ }; i < OBSTACLES_AMOUNT; i++) {
            int x{ range(generator) };
            int y{ range(generator) };
            auto* node = GetNode(x, y);

            if (node == m_start || node == m_end)
                continue;

            if (node->IsObstacle)
                continue;

            node->IsObstacle = true;
        }
    }

    void Update() {
        if (m_finished) return;

        if (m_opened.empty()) {
            m_finished = true;
            return;
        }

        auto current = m_opened.top();
        m_opened.pop();

        if (current.m_Node->IsClosed)
            return;

        current.m_Node->IsClosed = true;

        if (current.m_Node == m_end) {
            ReconstructPath();
            m_finished = true;
            return;
        }

        ExpandNode(current.m_Node);
    }

    void Draw() {
        for (int y{}; y < NODE_ROW_AMOUNT; y++) {
            for (int x{}; x < NODE_ROW_AMOUNT; x++) {
                auto* node = GetNode(x, y);
                Color color = node->IsObstacle ? BLACK : Color(150, 150, 150, 150);

                if (node->IsClosed)
                    color = BLUE;
                else if (node->IsOpen)
                    color = DARKBLUE;

                const int px = x * NODE_PX_SIZE;
                const int py = y * NODE_PX_SIZE;
                DrawRectangle(px, py, NODE_PX_SIZE, NODE_PX_SIZE, color);
                DrawRectangleLines(px, py, NODE_PX_SIZE, NODE_PX_SIZE, GRAY);
            }
        }

        for (const auto* node : m_finalPath) {
            DrawRectangle(node->X * NODE_PX_SIZE, node->Y * NODE_PX_SIZE, NODE_PX_SIZE, NODE_PX_SIZE, ORANGE);
        }

        DrawRectangle(m_start->X * NODE_PX_SIZE, m_start->Y * NODE_PX_SIZE, NODE_PX_SIZE, NODE_PX_SIZE, GREEN);
        DrawRectangle(m_end->X * NODE_PX_SIZE, m_end->Y * NODE_PX_SIZE, NODE_PX_SIZE, NODE_PX_SIZE, RED);
    }

private:
    struct Node {
        int X{};
        int Y{};

        float HCost{};
        float GCost{std::numeric_limits<float>::max()};
        float FCost{ std::numeric_limits<float>::max() };

        bool IsObstacle{};
        bool IsOpen{};
        bool IsClosed{};

        Node* Parent;
    };

    struct OpenNode {
        Node* m_Node{};
    
        bool operator<(const OpenNode& other) const
        {
            return m_Node->FCost > other.m_Node->FCost;
        }
    };

    float heuristic(const Node* a, const Node* b) {
        return static_cast<float>(std::abs(a->X - b->X) + std::abs(a->Y - b->Y));
    }

    Node* GetNode(float x, float y)  {
        if (x < 0 || y < 0) {
            return nullptr;
        }
        if (x > NODE_ROW_AMOUNT - 1 || y > NODE_ROW_AMOUNT - 1) {
            return nullptr;
        }

        return &m_grid[y][x];
    }
    
    void ReconstructPath() {
        auto* current{ m_end };

        while (current->Parent) {
            m_finalPath.push_back(current);
            current = current->Parent;
        }
        m_finalPath.push_back(current);
    }
    void ExpandNode(Node* node) {
        node->IsClosed = true;

        for (auto* neighbour : GetNeighbours(node)) {
            const float newGCost = node->GCost + 1;
            if (newGCost < neighbour->GCost) {
                neighbour->GCost = newGCost;
                neighbour->HCost = heuristic(neighbour, m_end);
                neighbour->FCost = neighbour->GCost + neighbour->FCost;
                neighbour->IsOpen = true;
                neighbour->Parent = node;

                m_opened.push(OpenNode(neighbour));
            }
        }
    }
    std::vector<Node*> GetNeighbours(const Node* node) {
       const std::vector<Vector2> dirs = { {0, 1}, {0, -1} , {1, 0}, {-1, 0} };
       std::vector<Node*> output{};

       for (const auto& direction : dirs) {
           auto* neighbour = GetNode(node->X + direction.x,node->Y + direction.y);

          if (!neighbour || neighbour->IsObstacle) continue;

          output.push_back(neighbour);
       }
       return output;
    }


    std::array<std::array<Node, NODE_ROW_AMOUNT>, NODE_ROW_AMOUNT> m_grid{};
    std::vector<Node*> m_finalPath{};
    std::priority_queue<OpenNode> m_opened{};

    Node* m_start{};
    Node* m_end{};
    bool m_finished{};
};

int main() {
	InitWindow(1000, 1000, "A* Algorithm");
    SetTargetFPS(60);

    std::unique_ptr<AStarAlgorithm> astar = std::make_unique<AStarAlgorithm>();

    while (!WindowShouldClose())
    {
        if (GetKeyPressed() == KEY_R) {
            astar = std::make_unique<AStarAlgorithm>();
        }
           

        astar->Update();
        BeginDrawing();
        ClearBackground(BLACK);
        astar->Draw();
        EndDrawing();
    }

	CloseWindow();
	return 0;
}
