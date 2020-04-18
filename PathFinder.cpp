#include "PathFinder.h"

#include "Misc/Assertx.h"

#include <algorithm>
#include <cassert>

namespace
{
    class NodePrioritizer
    {
public:
        // Return true if x is higher cost than y (which means it has lower priority)
        bool operator ()(PathFinder::Node const * x, PathFinder::Node const * y) const
        {
            return x->f > y->f;
        }
    };
}

//! @param  domain  Path nodes (if nullptr, the nodes must be explicitly reset before calling findPath)
//! @param  policy  Configuration options

PathFinder::PathFinder(NodeList * domain, Policy const & policy)
    : domain_(domain)
    , policy_(policy)
{
}

//! @param    start     Start node
//! @param    end       End node
//! @param    path      Resulting path
//!
//! @returns    true, if a path is found

bool PathFinder::findPath(Node * start, Node * end, Path * path)
{
#if defined(_DEBUG)
    validateNode(start);
    validateNode(end);
#endif

    std::vector<Node *> open;
    if (policy_.maxNodes > 0)
        open.reserve(policy_.maxNodes);

    // Reset the status of all nodes

    resetNodes();

    // Add the start node to the open queue.

    assert(policy_.maxNodes <= 0 || open.size() < open.capacity());
    start->open(0.f, nullptr, *end);
    open.push_back(start);

    // Until the open queue is empty or a path is found...

    while (!open.empty())
    {
        // Get the lowest cost node as the next one to check (and close it)

        Node * pNode = open.front();

        pNode->close();
        pop_heap(open.begin(), open.end(), NodePrioritizer());
        open.pop_back();

        // If this is the goal, then we are done

        if (pNode == end)
        {
            constructPath(start, end, path);
            return true;
        }

        // Go to each neighbor and set/update its cost and make sure it is in the open queue (unless it is closed)

        EdgeList & edges = pNode->adjacencies;

        for (auto const & edge : edges)
        {
            Node * pNeighbor = edge->to;

#if defined(_DEBUG)
            validateNode(pNeighbor);
#endif

            // If the node is closed, then its minimum cost has been determined and there is no reason to revisit
            // it.

            if (pNeighbor->isClosed())
                continue;

            // Compute the cost to the neighbor through this node
            float cost = pNode->g + edge->cost;

            // If the neighbor is not in the open queue, then add it

            if (!pNeighbor->isOpen())
            {
                pNeighbor->open(cost, pNode, *end);

                // If the open queue is full then remove the last entry to make room. The last entry is not
                // necessarily the highest cost node, but it is guaranteed to be in the highest 50%. The
                // non-determinism is the price to pay for a fixed-size queue.

                if (policy_.maxNodes > 0 && policy_.maxNodes <= (int)open.size())
                {
                    open.back()->close();
                    open.pop_back();
                }

                assert(policy_.maxNodes <= 0 || open.size() < open.capacity());
                open.push_back(pNeighbor);
                push_heap(open.begin(), open.end(), NodePrioritizer());
            }

            // Otherwise, perhaps this is a lower-cost path to it. If so, update it to reflect the new path.

            else if (cost < pNeighbor->g)
            {
                pNeighbor->update(cost, pNode);
                make_heap(open.begin(), open.end(), NodePrioritizer());     // Ouch. This could be expensive.
                                                                            // If we knew where in the heap this
                                                                            // value is we could push_heap instead.
            }
        }
    }

    return false;
}

void PathFinder::resetNodes()
{
    if (domain_)
    {
        for (auto const & node : *domain_)
        {
            node->reset();
        }
    }
}

void PathFinder::constructPath(Node * from, Node * to, Path * path)
{
    assert(path);

    // Make sure the path is empty
    path->clear();

    // The nodes are linked from end to start, so the vector is filled in that order, and then the order of the
    // elements is the vector is reversed.

    // Add nodes to the path (end-to-start)

    for (Node * pNode = to; pNode != nullptr; pNode = pNode->predecessor)
    {
        path->push_back(pNode);
    }

    assert(path->back() == from);

    // Reverse to start-to-end

    reverse(path->begin(), path->end());
}

#if defined(_DEBUG)

void PathFinder::validateNode(Node * node)
{
    // Assert that the node is in the domain (if one is provided)
    assert(domain_ == nullptr || find(domain_->begin(), domain_->end(), node) != domain_->end());
}

#endif // defined( _DEBUG )

void PathFinder::Node::open(float cost, Node * pPredecessor, Node const & goal)
{
    assert(status_ == Status::NOT_VISITED);
    cachedH_ = h(goal);
    update(cost, pPredecessor);
    status_ = Status::OPEN;
}

void PathFinder::Node::close()
{
    assert(status_ == Status::OPEN);
    status_ = Status::CLOSED;
}

void PathFinder::Node::reset()
{
    f           = 0.0f;
    g           = 0.0f;
    predecessor = nullptr;
    cachedH_    = 0.0f;
    status_     = Status::NOT_VISITED;
}

void PathFinder::Node::update(float cost, Node * pred)
{
    g           = cost;
    f           = g + cachedH_;
    predecessor = pred;
}
