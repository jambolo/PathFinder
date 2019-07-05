/** @file *//********************************************************************************************************

                                                    PathFinder.h

						                    Copyright 2008, John J. Bolton
	--------------------------------------------------------------------------------------------------------------

	$Header: //depot/Libraries/PathFinder/PathFinder.h#4 $

	$NoKeywords: $

*********************************************************************************************************************/

#pragma once


#include <vector>


/********************************************************************************************************************/

class PathFinder
{
public:

	class Node;
	class Edge;

	typedef std::vector< Node * > Path;
	typedef std::vector< Node * > Domain;
	typedef std::vector< Edge * > EdgeList;

	struct Policy
	{
		int	maxNodes;	//!< Maximum number of nodes to be used in the search (or <= 0 for unlimited)

		Policy( int _maxNodes ) : maxNodes( _maxNodes )	{}
	};

	PathFinder( Domain * pDomain, Policy const & policy );

	//! Finds the shortest path. Returns true if a path was found.
	bool FindPath( Node * pStart, Node * pEnd, Path * pPath );

private:

	// Reset the status of all nodes in the domain
	void _resetNodes();
	
	// Constructs the path
	void _constructPath( Node * pFrom, Node *pTo, Path * pPath );

#if defined( _DEBUG )
	void _validateNode( Node * pNode );
#endif

	Domain *	mpDomain;
	Policy		mPolicy;
};


/********************************************************************************************************************/

class PathFinder::Node
{
public:

	Node();

	//! @brief	Returns the estimated cost from this node to the goal.
	//! @note	This value is assumed to be constant.
	virtual float h( Node const & goal ) const = 0;

	//! Updates pathfinding values for the node
	void Update( float cost, Node * pPredecessor );

	//! Anticipates being added to the open queue
	void Open( float cost, Node * pPredecessor, Node const & goal );

	//! Returns true if the node is open.
	bool IsOpen() const		{ return m_Status == STATUS_OPEN; }

	//! Anticipates being closed.
	void Close();

	//! Returns true if the node is closed
	bool IsClosed() const	{ return m_Status == STATUS_CLOSED; }

	//! Resets the node's status to not visited
	void Reset();

	PathFinder::EdgeList	m_AdjacencyList;

	// For use by the pathfinder

	float	m_F;				//!< Estimated cost of total path through this node
	float	m_G;				//!< Cost of path to this node
	Node *	m_pPredecessor;		//!< Previous node in the path (assuming the path goes through this node)

private:

	enum Status
	{
		STATUS_NOT_VISITED,
		STATUS_OPEN,
		STATUS_CLOSED
	};

	float	m_CachedH;	// Cached value of h()
	Status	m_Status;	// Node status
};


/********************************************************************************************************************/

class PathFinder::Edge
{
public:

	Edge()	{}
	Edge( float cost );

	float				m_Cost;
	PathFinder::Node *	m_pTo;
};
