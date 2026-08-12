#pragma once
#include <limits>

struct Node {
    int X{};
    int Y{};

    float HCost{};
    float GCost{ std::numeric_limits<float>::max() };
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