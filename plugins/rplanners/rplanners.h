// -*- coding: utf-8 -*-
// Copyright (C) 2006-2014 Rosen Diankov <rosen.diankov@gmail.com>
//
// This file is part of OpenRAVE.
// OpenRAVE is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
#ifndef RAVE_PLANNERS_H
#define RAVE_PLANNERS_H

#include "openraveplugindefs.h"

#include <boost/pool/pool.hpp>

#define _(msgid) OpenRAVE::RaveGetLocalizedTextForDomain("openrave_plugins_rplanners", msgid)

enum ExtendType {
    ET_Failed=0,
    ET_Sucess=1,
    ET_Connected=2
};

#ifndef __clang__
/// \brief wraps a static array of T onto a std::vector. Destructor just NULLs out the pointers. Any dynamic resizing operations on this vector wrapper would probably cause the problem to segfault, so use as if it is constant.
///
/// This is an optimization, so if there's a compiler this doesn't compile for, #ifdef it with the regular vector.
template <class T>
class VectorWrapper : public std::vector<T>
{
public:
    VectorWrapper() {
        this->_M_impl._M_start = this->_M_impl._M_finish = this->_M_impl._M_end_of_storage = NULL;
    }

    VectorWrapper(T* sourceArray, T* sourceArrayEnd)
    {
        this->_M_impl._M_start = sourceArray;
        this->_M_impl._M_finish = this->_M_impl._M_end_of_storage = sourceArrayEnd;
    }

    // dangerous! user has to make sure not to modify anything...
    VectorWrapper(const T* sourceArray, const T* sourceArrayEnd)
    {
        this->_M_impl._M_start = const_cast<T*>(sourceArray);
        this->_M_impl._M_finish = this->_M_impl._M_end_of_storage = const_cast<T*>(sourceArrayEnd);
    }

    ~VectorWrapper() {
        this->_M_impl._M_start = this->_M_impl._M_finish = this->_M_impl._M_end_of_storage = NULL;
    }

    void WrapArray(T* sourceArray, int arraySize)
    {
        this->_M_impl._M_start = sourceArray;
        this->_M_impl._M_finish = this->_M_impl._M_end_of_storage = sourceArray + arraySize;
    }
};
#else // __clang__
#define VectorWrapper std::vector
#endif // __clang__

class SimpleNode;
using SimpleNodePtr = SimpleNode*;

/// \brief node in freespace. be careful when constructing since placement new operator is needed.
class SimpleNode
{
public:
    SimpleNode(SimpleNodePtr parent, const std::vector<dReal>& config);
    SimpleNode(SimpleNodePtr parent, const dReal* pconfig, int dof);
    ~SimpleNode();

    SimpleNodePtr rrtparent; ///< pointer to the RRT tree parent
    std::vector<SimpleNodePtr> _vchildren; ///< cache tree direct children of this node (for the next cache level down). Has nothing to do with the RRT tree.
    int16_t _level = 0; ///< the level the node belongs to
    uint8_t _hasselfchild = 0; ///< if 1, then _vchildren has contains a clone of this node in the level below it.
    uint8_t _usenn = 1; ///< if 1, then use part of the nearest neighbor search, otherwise ignore
    uint32_t _userdata = 0; ///< user specified data tagging this node

#ifdef _DEBUG
    int id;
#endif
    dReal q[0]; // the configuration immediately follows the struct
};

using DistMetricFn = boost::function<dReal(const std::vector<dReal>&, const std::vector<dReal>&)>;

class SpatialTreeBase
{
public:
    virtual void Init(PlannerBaseWeakPtr planner,
                      const int dof,
                      DistMetricFn& distmetricfn,
                      const dReal fStepLength,
                      const dReal maxdistance) = 0;

    /// inserts a node in the try
    virtual SimpleNodePtr InsertNode(SimpleNodePtr parent,
                                     const std::vector<dReal>& config,
                                     uint32_t userdata) = 0;

    /// returns the nearest neighbor
    virtual std::pair<SimpleNodePtr, dReal> FindNearestNode(const std::vector<dReal>& q) const = 0;

    /// \brief returns a temporary config stored on the local class. Next time this function is called, it will overwrite the config
    virtual const std::vector<dReal>& GetVectorConfig(SimpleNodePtr node) const = 0;

    virtual void GetVectorConfig(SimpleNodePtr nodebase,
                                 std::vector<dReal>& v) const = 0;

    /// extends toward pNewConfig
    /// \return true if extension reached pNewConfig
    virtual ExtendType Extend(const std::vector<dReal>& pTargetConfig,
                              SimpleNodePtr& lastnode,
                              bool bOneStep = false) = 0;

    /// \brief the dof configured for
    virtual int GetDOF() = 0;
    virtual bool Validate() const = 0;
    virtual int GetNumNodes() const = 0;

    /// invalidates any nodes that point to parentbase. nodes can still be references from outside, but just won't be used as part of the nearest neighbor search
    virtual void InvalidateNodesWithParent(SimpleNodePtr parentbase) = 0;
};

/// Cache stores configuration information in a data structure based on the Cover Tree (Beygelzimer et al. 2006 http://hunch.net/~jl/projects/cover_tree/icml_final/final-icml.pdf)
class SpatialTree : public SpatialTreeBase
{
public:
    SpatialTree(int fromgoal);
    ~SpatialTree();

    /* =========== Overridden functions ========== */
    virtual void Init(PlannerBaseWeakPtr planner,
                      const int dof,
                      DistMetricFn& distmetricfn,
                      const dReal fStepLength,
                      const dReal maxdistance) override;
    virtual SimpleNodePtr InsertNode(SimpleNodePtr parent,
                                     const std::vector<dReal>& config,
                                     uint32_t userdata) override;
    virtual std::pair<SimpleNodePtr, dReal> FindNearestNode(const std::vector<dReal>& vquerystate) const override;
    virtual const std::vector<dReal>& GetVectorConfig(SimpleNodePtr nodebase) const override;
    virtual void GetVectorConfig(SimpleNodePtr nodebase, std::vector<dReal>& v) const override;
    virtual ExtendType Extend(const vector<dReal>& vTargetConfig, SimpleNodePtr& lastnode, bool bOneStep=false) override;
    virtual int GetNumNodes() const override;
    virtual int GetDOF() override;
    /// \brief for debug purposes, validates the tree
    virtual bool Validate() const override;
    virtual void InvalidateNodesWithParent(SimpleNodePtr parentbase) override;

    /* =========== Own functions ========== */
    bool empty() const;
    void DumpTree(std::ostream& o) const;

    /// \brief given random index 0 <= inode < _numnodes, return a node. If tree changes, indices might change
    SimpleNodePtr GetNodeFromIndex(size_t inode) const;
    void GetNodesVector(std::vector<SimpleNodePtr>& vnodes) const;

private:
    void _Reset();
    static int GetNewStaticId();
    SimpleNodePtr _CreateNode(SimpleNodePtr rrtparent, const std::vector<dReal>& config, uint32_t userdata);

    SimpleNodePtr _CloneNode(SimpleNodePtr refnode);

    dReal _ComputeDistance(const dReal* config0, const dReal* config1) const;
    dReal _ComputeDistance(const dReal* config0, const std::vector<dReal>& config1) const;
    dReal _ComputeDistance(SimpleNodePtr node0, SimpleNodePtr node1) const;
    dReal _ComputeDistance(const std::vector<dReal>& v0,
                           const std::vector<dReal>& v1) const;

    void _DeleteNode(SimpleNodePtr p);
    /// deletes all nodes that have parentindex as their parent
    void _DeleteNodesWithParent(SimpleNodePtr parentbase);

    int _EncodeLevel(int level) const;

    std::pair<SimpleNodePtr, dReal> _FindNearestNode(const std::vector<dReal>& vquerystate) const;

    SimpleNodePtr _InsertNode(SimpleNodePtr parent, const std::vector<dReal>& config, uint32_t userdata);

    /// \brief the recursive function that inserts a configuration into the cache tree
    ///
    /// \param[in] nodein the input node to insert
    /// \param[in] vCurrentLevelNodes the tree nodes at "level" with the respecitve distances computed for them
    /// \param[in] currentlevel the current level traversing
    /// \param[in] fLevelBound pow(_base, level)
    /// \return 1 if point is inserted and parent found. 0 if no parent found and point is not inserted. -1 if parent found but point not inserted since it is close to _mindistance
    int _InsertRecursive(SimpleNodePtr nodein,
                         const std::vector< std::pair<SimpleNodePtr, dReal> >& vCurrentLevelNodes,
                         int currentlevel,
                         dReal fLevelBound);

    /// \brief inerts a node directly to parentnode
    ///
    /// If parentnode's configuration is too close to nodein, or parentnode's level is too high, will create dummy child nodes
    bool _InsertDirectly(SimpleNodePtr nodein, SimpleNodePtr parentnode, dReal parentdist, int maxinsertlevel, dReal fInsertLevelBound);

    bool _RemoveNode(SimpleNodePtr removenode);

    bool _Remove(SimpleNodePtr removenode, std::vector< std::vector<SimpleNodePtr> >& vvCoverSetNodes, int currentlevel, dReal fLevelBound);

    DistMetricFn _distmetricfn;
    PlannerBaseWeakPtr _planner;
    dReal _fStepLength = 0.04;
    int _dof = 0; ///< the number of values of each state
    int _fromgoal = 0; ///< either _treeForward(0) in RrtPlanner, or _treeBackward(1) in BirrtPlanner that inherits RrtPlanner

    // cover tree data structures
    boost::shared_ptr< boost::pool<> > _pNodesPool; ///< pool nodes are created from

    std::vector< std::set<SimpleNodePtr> > _vsetLevelNodes; ///< _vsetLevelNodes[enc(level)][node] holds the indices of the children of "node" of a given the level. enc(level) maps (-inf,inf) into [0,inf) so it can be indexed by the vector. Every node has an entry in a map here. If the node doesn't hold any children, then it is at the leaf of the tree. _vsetLevelNodes.at(_EncodeLevel(_maxlevel)) is the root.

    dReal _maxdistance = 0.0; ///< maximum possible distance between two states. used to balance the tree. Has to be > 0.
    dReal _mindistance = 0.0; ///< minimum possible distance between two states until they are declared the same
    dReal _base = 1.5; ///< a constant used to control the max level of traversion
    dReal _fBaseInv = 2.0/3.0; ///< _fBaseInv = 1/_base
    dReal _fBaseChildMult = 2.0; ///< _fBaseChildMult = 1/(_base-1)
    int _maxlevel = 0; ///< the maximum allowed levels in the tree, this is where the root node starts (inclusive)
    int _minlevel = 0; ///< the minimum allowed levels in the tree (inclusive)
    int _numnodes = 0; ///< the number of nodes in the current tree starting at the root at _vsetLevelNodes.at(_EncodeLevel(_maxlevel))
    dReal _fMaxLevelBound = 0.0; // pow(_base, _maxlevel)

    // cache
    std::vector<SimpleNodePtr> _vchildcache;
    std::set<SimpleNodePtr> _setchildcache;
    std::vector<dReal> _vNewConfig, _vDeltaConfig, _vCurConfig;
    mutable std::vector<dReal> _vTempConfig;
    ConstraintFilterReturnPtr _constraintreturn;

    mutable std::vector< std::pair<SimpleNodePtr, dReal> > _vCurrentLevelNodes;
    mutable std::vector< std::pair<SimpleNodePtr, dReal> > _vNextLevelNodes;
    mutable std::vector< std::vector<SimpleNodePtr> > _vvCacheNodes;
};

#ifdef RAVE_REGISTER_BOOST
#include BOOST_TYPEOF_INCREMENT_REGISTRATION_GROUP()
BOOST_TYPEOF_REGISTER_TYPE(SimpleNode)
BOOST_TYPEOF_REGISTER_TYPE(SpatialTree)
#endif

#endif
