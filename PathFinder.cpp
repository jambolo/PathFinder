/** @file *//********************************************************************************************************

                                                 PathFinder.cpp

						                    Copyright 2008, John J. Bolton
	--------------------------------------------------------------------------------------------------------------

	$Header: //depot/Libraries/PathFinder/PathFinder.cpp#5 $

	$NoKeywords: $

*********************************************************************************************************************/

#include "PathFinder.h"

#include "Misc/Assert.h"

#include <algorithm>


namespace
{
	class NodePrioritizer
	{
	public:
		// Return true if x is higher cost than y (which means it has lower priority)
		bool operator ()( PathFinder::Node const * x, PathFinder::Node const * y ) const
		{
			return ( x->m_F > y->m_F );
		}
	};

}

/********************************************************************************************************************/

//! @param		pDomain			Path nodes (if NULL, the nodes must be explicitly reset before calling FindPath)
//!	@param		policy			Configuration options

PathFinder::PathFinder( Domain * pDomain, Policy const & policy )
	: mpDomain( pDomain ), mPolicy( policy )
{
}

/********************************************************************************************************************/

//! @param	pStart		Start node
//! @param	pEnd		End node
//! @param	pPath		Resulting path
//!
//! @returns	true, if a path is found

bool PathFinder::FindPath( Node * pStart, Node * pEnd, Path * pPath )
{
#if defined( _DEBUG )
	_validateNode( pStart );
	_validateNode( pEnd );
#endif

	std::vector<Node *>	open;
	if ( mPolicy.maxNodes > 0 )
	{
		open.reserve( mPolicy.maxNodes );
	}

	// Reset the status of all nodes

	_resetNodes();

	// Add the start node to the open queue.

	assert( mPolicy.maxNodes <= 0 || open.size() < open.capacity() );
	pStart->Open( 0.f, NULL, *pEnd );
	open.push_back( pStart );

	// Until the open queue is empty or a path is found...

	while ( !open.empty() )
	{
		// Get the lowest cost node as the next one to check (and close it)

		Node * pNode	= open.front();

		pNode->Close();
		pop_heap( open.begin(), open.end(), NodePrioritizer() );
		open.pop_back();

		// If this is the goal, then we are done

		if ( pNode == pEnd )
		{
			_constructPath( pStart, pEnd, pPath );
			return true;
		}

		// Go to each neighbor and set/update its cost and make sure it is in the open queue (unless it is closed)

		EdgeList &	edges			= pNode->m_AdjacencyList;

		for ( EdgeList::iterator ppEdge = edges.begin(); ppEdge != edges.end(); ++ppEdge )
		{
			Edge *	pEdge		= *ppEdge;
			Node *	pNeighbor	= pEdge->m_pTo;

#if defined( _DEBUG )
			_validateNode( pNeighbor );
#endif

			// If the node is closed, then its minimum cost has been determined and there is no reason to revisit
			// it.

			if ( pNeighbor->IsClosed() )
			{
				continue;
			}

			// Compute the cost to the neighbor through this node
			float	cost	= pNode->m_G + pEdge->m_Cost;

			// If the neighbor is not in the open queue, then add it

			if ( !pNeighbor->IsOpen() )
			{
				pNeighbor->Open( cost, pNode, *pEnd );

				// If the open queue is full then remove the last entry to make room. The last entry is not
				// necessarily the highest cost node, but it is guaranteed to be in the highest 50%. The
				// non-determinism is the price to pay for a fixed-size queue.

				if ( mPolicy.maxNodes > 0 && mPolicy.maxNodes <= (int)open.size() )
				{
					open.back()->Close();
					open.pop_back();
				}

				assert( mPolicy.maxNodes <= 0 || open.size() < open.capacity() );
				open.push_back( pNeighbor );
				push_heap( open.begin(), open.end(), NodePrioritizer() );
			}

			// Otherwise, perhaps this is a lower-cost path to it. If so, update it to reflect the new path.

			else if ( cost < pNeighbor->m_G )
			{
				pNeighbor->Update( cost, pNode );
				make_heap( open.begin(), open.end(), NodePrioritizer() );	// Ouch. This could be expensive.
																			// If we knew where in the heap this
																			// value is we could push_heap instead.
			}
		}
	}

	return false;
}


/********************************************************************************************************************/

void PathFinder::_resetNodes()
{
	if ( mpDomain != NULL )
	{
		for ( Domain::iterator ppNode = mpDomain->begin(); ppNode != mpDomain->end(); ++ppNode )
		{
			Node *	pNode	= *ppNode;
			pNode->Reset();
		}
	}
}


/********************************************************************************************************************/

void PathFinder::_constructPath( Node * pStart, Node * pEnd, Path * pPath )
{
	// Make sure the path is empty
	pPath->clear();

	// The nodes are linked from end to start, so the vector is filled in that order, and then the order of the
	// elements is the vector is reversed.

	// Add nodes to the path (end-to-start)

	for ( Node * pNode = pEnd; pNode != NULL; pNode = pNode->m_pPredecessor )
	{
		pPath->push_back( pNode );
	}

	assert( pPath->back() == pStart );

	// Reverse to start-to-end

	reverse( pPath->begin(), pPath->end() );
}


/********************************************************************************************************************/

#if defined( _DEBUG )

void PathFinder::_validateNode( Node * pNode )
{
	// Assert that the node is in the domain (if one is provided)
	assert( mpDomain == NULL || find( mpDomain->begin(), mpDomain->end(), pNode ) != mpDomain->end() );
}

#endif // defined( _DEBUG )


/********************************************************************************************************************/

void PathFinder::Node::Open( float cost, Node * pPredecessor, Node const & goal )
{
	assert( m_Status == STATUS_NOT_VISITED );
	m_CachedH = h( goal );
	Update( cost, pPredecessor );
	m_Status = STATUS_OPEN;
}


/********************************************************************************************************************/

void PathFinder::Node::Close()
{
	assert( m_Status == STATUS_OPEN );
	m_Status = STATUS_CLOSED;
}


/********************************************************************************************************************/

void PathFinder::Node::Reset()
{
	m_F				= 0.0f;
	m_G				= 0.0f;
	m_pPredecessor	= NULL;
	m_CachedH		= 0.0f;
	m_Status		= STATUS_NOT_VISITED;
}


/********************************************************************************************************************/

void PathFinder::Node::Update( float cost, Node * pPredecessor )
{
	m_G = cost;
	m_F = m_G + m_CachedH;
	m_pPredecessor = pPredecessor;
}


/********************************************************************************************************************/

PathFinder::Edge::Edge( float cost )
	: m_Cost( cost )
{
}
