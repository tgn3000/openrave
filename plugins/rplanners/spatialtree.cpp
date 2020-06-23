#include "rplanners.h"

SimpleNode::SimpleNode(SimpleNode* parent, const vector<dReal>& config) : rrtparent(parent) {
    std::copy(config.begin(), config.end(), q);
    _level = 0;
    _hasselfchild = 0;
    _usenn = 1;
    _userdata = 0;
}

SimpleNode::SimpleNode(SimpleNode* parent, const dReal* pconfig, int dof) : rrtparent(parent) {
    std::copy(pconfig, pconfig+dof, q);
    _level = 0;
    _hasselfchild = 0;
    _usenn = 1;
    _userdata = 0;
}

SimpleNode::~SimpleNode() {
}

/// Cache stores configuration information in a data structure based on the Cover Tree (Beygelzimer et al. 2006 http://hunch.net/~jl/projects/cover_tree/icml_final/final-icml.pdf)
SpatialTree::SpatialTree(int fromgoal) {
    _fromgoal = fromgoal;
}

SpatialTree::~SpatialTree() {
    this->Reset();
}

void SpatialTree::Init(PlannerBaseWeakPtr planner, int dof, DistMetricFn& distmetricfn, dReal fStepLength, dReal maxdistance)
{
    this->Reset();
    if( !!_pNodesPool ) {
        // see if pool can be preserved
        if( _dof != dof ) {
            _pNodesPool.reset();
        }
    }
    if( !_pNodesPool ) {
        _pNodesPool.reset(new boost::pool<>(sizeof(SimpleNode)+dof*sizeof(dReal)));
    }
    _planner = planner;
    _distmetricfn = distmetricfn;
    _fStepLength = fStepLength;
    _dof = dof;
    _vNewConfig.resize(dof);
    _vDeltaConfig.resize(dof);
    _vTempConfig.resize(dof);
    _maxdistance = maxdistance;
    _mindistance = 0.001*fStepLength; ///< is it ok?
    _maxlevel = ceilf(RaveLog(_maxdistance)/RaveLog(_base));
    _minlevel = _maxlevel - 1;
    _fMaxLevelBound = RavePow(_base, _maxlevel);
    int enclevel = _EncodeLevel(_maxlevel);
    if( enclevel >= (int)_vsetLevelNodes.size() ) {
        _vsetLevelNodes.resize(enclevel+1);
    }
    _constraintreturn.reset(new ConstraintFilterReturn());
}

void SpatialTree::Reset()
{
    if( !!_pNodesPool ) {
        // make sure all children are deleted
        for(std::set<SimpleNodePtr>& sLevelNodes : _vsetLevelNodes) {
            for(SimpleNodePtr node : sLevelNodes) {
                node->~SimpleNode();
            }
        }
        for(std::set<SimpleNodePtr>& sLevelNodes : _vsetLevelNodes) {
            sLevelNodes.clear();
        }
        _pNodesPool.reset(new boost::pool<>(sizeof(SimpleNode)+_dof*sizeof(dReal)));
    }
    _numnodes = 0;
}

dReal SpatialTree::_ComputeDistance(const dReal* config0, const dReal* config1) const
{
    return _distmetricfn(VectorWrapper<dReal>(config0, config0+_dof), VectorWrapper<dReal>(config1, config1+_dof));
}

dReal SpatialTree::_ComputeDistance(const dReal* config0, const std::vector<dReal>& config1) const
{
    return _distmetricfn(VectorWrapper<dReal>(config0,config0+_dof), config1);
}

dReal SpatialTree::_ComputeDistance(SimpleNodePtr node0, SimpleNodePtr node1) const
{
    return _distmetricfn(VectorWrapper<dReal>(node0->q, &node0->q[_dof]), VectorWrapper<dReal>(node1->q, &node1->q[_dof]));
}

std::pair<SimpleNodePtr, dReal> SpatialTree::FindNearestNode(const std::vector<dReal>& vquerystate) const
{
    return _FindNearestNode(vquerystate);
}

SimpleNodePtr SpatialTree::InsertNode(SimpleNodePtr parent, const vector<dReal>& config, uint32_t userdata)
{
    return _InsertNode(parent, config, userdata);
}

void SpatialTree::InvalidateNodesWithParent(SimpleNodePtr parentbase)
{
    //BOOST_ASSERT(Validate());
    const uint64_t starttime = utils::GetNanoPerformanceTime();
    // first gather all the nodes, and then delete them in reverse order they were originally added in
    SimpleNodePtr parent = parentbase;
    parent->_usenn = 0;
    _setchildcache.clear(); _setchildcache.insert(parent);
    int numruns=0;
    bool bchanged=true;
    while(bchanged) {
        bchanged=false;
        for(std::set<SimpleNodePtr>& sLevelNodes : _vsetLevelNodes) {
            for(SimpleNodePtr node : sLevelNodes) {
                if( _setchildcache.find(node) == _setchildcache.end() && _setchildcache.find(node->rrtparent) != _setchildcache.end() ) {
                    node->_usenn = 0;
                    _setchildcache.insert(node);
                    bchanged=true;
                }
            }
        }
        ++numruns;
    }
    RAVELOG_VERBOSE("computed in %fs", (1e-9*(utils::GetNanoPerformanceTime()-starttime)));
}

/// deletes all nodes that have parentindex as their parent
void SpatialTree::_DeleteNodesWithParent(SimpleNodePtr parentbase)
{
    BOOST_ASSERT(Validate());
    uint64_t starttime = utils::GetNanoPerformanceTime();
    // first gather all the nodes, and then delete them in reverse order they were originally added in
    SimpleNodePtr parent = parentbase;
    if( _vchildcache.capacity() == 0 ) {
        _vchildcache.reserve(128);
    }
    _vchildcache.clear(); _vchildcache.push_back(parent);
    _setchildcache.clear(); _setchildcache.insert(parent);
    int numruns=0;
    bool bchanged=true;
    while(bchanged) {
        bchanged=false;
        FOREACHC(itchildren, _vsetLevelNodes) {
            FOREACHC(itchild, *itchildren) {
                if( _setchildcache.find(*itchild) == _setchildcache.end() && _setchildcache.find((*itchild)->rrtparent) != _setchildcache.end() ) {
                    //if( !std::binary_search(_vchildcache.begin(),_vchildcache.end(),(*itchild)->rrtparent) ) {
                    _vchildcache.push_back(*itchild);
                    _setchildcache.insert(*itchild);
                    bchanged=true;
                }
            }
        }
        ++numruns;
    }

    int nremove=0;
    // systematically remove backwards
    for(std::vector<SimpleNodePtr>::reverse_iterator itnode = _vchildcache.rbegin(); itnode != _vchildcache.rend(); ++itnode) {
        bool bremoved = _RemoveNode(*itnode);
        BOOST_ASSERT(bremoved);
        ++nremove;
    }
    BOOST_ASSERT(Validate());
    RAVELOG_VERBOSE("computed in %fs", (1e-9*(utils::GetNanoPerformanceTime()-starttime)));
}

ExtendType SpatialTree::Extend(const vector<dReal>& vTargetConfig, SimpleNodePtr& lastnode, bool bOneStep)
{
    // get the nearest neighbor
    std::pair<SimpleNodePtr, dReal> nn = _FindNearestNode(vTargetConfig);
    if( !nn.first ) {
        return ET_Failed;
    }
    SimpleNodePtr pnode = nn.first;
    lastnode = nn.first;
    bool bHasAdded = false;
    boost::shared_ptr<PlannerBase> planner(_planner);
    PlannerBase::PlannerParametersConstPtr params = planner->GetParameters();
    _vCurConfig.resize(_dof);
    std::copy(pnode->q, pnode->q+_dof, _vCurConfig.begin());
    // extend
    for(int iter = 0; iter < 100; ++iter) {     // to avoid infinite loops
        dReal fdist = _ComputeDistance(&_vCurConfig[0], vTargetConfig);
        if( fdist > _fStepLength ) {
            fdist = _fStepLength / fdist;
        }
        else if( fdist <= dReal(0.01) * _fStepLength ) {
            // return connect if the distance is very close
            return ET_Connected;
        }
        else {
            fdist = 1;
        }

        _vNewConfig = _vCurConfig;
        _vDeltaConfig = vTargetConfig;
        params->_diffstatefn(_vDeltaConfig, _vCurConfig);
        for(int i = 0; i < _dof; ++i) {
            _vDeltaConfig[i] *= fdist;
        }
        if( params->SetStateValues(_vNewConfig) != 0 ) {
            return bHasAdded ? ET_Sucess : ET_Failed;
        }
        if( params->_neighstatefn(_vNewConfig,_vDeltaConfig,_fromgoal ? NSO_GoalToInitial : 0) == NSS_Failed ) {
            return bHasAdded ? ET_Sucess : ET_Failed;
        }
        // it could be the case that the node didn't move anywhere, in which case we would go into an infinite loop
        if( _ComputeDistance(&_vCurConfig[0], _vNewConfig) <= dReal(0.01)*_fStepLength ) {
            return bHasAdded ? ET_Sucess : ET_Failed;
        }

        // necessary to pass in _constraintreturn since _neighstatefn can have constraints and it can change the interpolation. Use _constraintreturn->_bHasRampDeviatedFromInterpolation to figure out if something changed.
        if( _fromgoal ) {
            if( params->CheckPathAllConstraints(_vNewConfig, _vCurConfig, {}, {}, 0, IT_OpenEnd, 0xffff|CFO_FillCheckedConfiguration, _constraintreturn) != 0 ) {
                return bHasAdded ? ET_Sucess : ET_Failed;
            }
        }
        else {
            if( params->CheckPathAllConstraints(_vCurConfig, _vNewConfig, {}, {}, 0, IT_OpenStart, 0xffff|CFO_FillCheckedConfiguration, _constraintreturn) != 0 ) {
                return bHasAdded ? ET_Sucess : ET_Failed;
            }
        }

        int iAdded = 0;
        if( _constraintreturn->_bHasRampDeviatedFromInterpolation ) {
            // Since the path checked by CheckPathAllConstraints can be different from a straight line segment connecting _vNewConfig and _vCurConfig, we add all checked configurations along the checked segment to the tree.
            if( _fromgoal ) {
                // Need to add nodes to the tree starting from the one closest to the nearest neighbor. Since _fromgoal is true, the closest one is the last config in _constraintreturn->_configurations
                for(int iconfig = ((int)_constraintreturn->_configurations.size()) - _dof; iconfig >= 0; iconfig -= _dof) {
                    std::copy(_constraintreturn->_configurations.begin() + iconfig, _constraintreturn->_configurations.begin() + iconfig + _dof, _vNewConfig.begin());
                    SimpleNodePtr pnewnode = _InsertNode(pnode, _vNewConfig, 0); ///< set userdata to 0
                    if( !!pnewnode ) {
                        bHasAdded = true;
                        pnode = pnewnode;
                        lastnode = pnode;
                        ++iAdded;
                    }
                    else {
                        // RAVELOG_DEBUG_FORMAT("_constraintreturn has %d configurations, numadded=%d, _fromgoal=%d", (_constraintreturn->_configurations.size()/_dof)%iAdded%_fromgoal);
                        break;
                    }
                }
            }
            else {
                for(int iconfig = 0; iconfig+_dof-1 < (int)_constraintreturn->_configurations.size(); iconfig += _dof) {
                    std::copy(_constraintreturn->_configurations.begin() + iconfig, _constraintreturn->_configurations.begin() + iconfig + _dof, _vNewConfig.begin());
                    SimpleNodePtr pnewnode = _InsertNode(pnode, _vNewConfig, 0); ///< set userdata to 0
                    if( !!pnewnode ) {
                        bHasAdded = true;
                        pnode = pnewnode;
                        lastnode = pnode;
                        ++iAdded;
                    }
                    else {
                        // RAVELOG_DEBUG_FORMAT("_constraintreturn has %d configurations, numadded=%d, _fromgoal=%d", (_constraintreturn->_configurations.size()/_dof)%iAdded%_fromgoal);
                        break;
                    }
                }
            }
        }
        else {
            SimpleNodePtr pnewnode = _InsertNode(pnode, _vNewConfig, 0); ///< set userdata to 0
            if( !!pnewnode ) {
                pnode = pnewnode;
                lastnode = pnode;
                bHasAdded = true;
            }
        }

        if( bHasAdded && bOneStep ) {
            return ET_Connected; // is it ok to return ET_Connected rather than ET_Sucess. BasicRRT relies on ET_Connected
        }
        _vCurConfig.swap(_vNewConfig);
    }

    return bHasAdded ? ET_Sucess : ET_Failed;
}

int SpatialTree::GetNumNodes() const {
    return _numnodes;
}

bool SpatialTree::empty() const {
    return _numnodes == 0;
}

const std::vector<dReal>& SpatialTree::GetVectorConfig(SimpleNodePtr node) const
{
    return _vTempConfig = std::vector<dReal>(node->q, node->q+_dof);
}

void SpatialTree::GetVectorConfig(SimpleNodePtr node, std::vector<dReal>& v) const
{
    v = std::vector<dReal>(node->q, node->q+_dof);
}

int SpatialTree::GetDOF() {
    return _dof;
}

/// \brief for debug purposes, validates the tree
bool SpatialTree::Validate() const
{
    if( _numnodes == 0 ) {
        return true;
    }

    if( _vsetLevelNodes.at(_EncodeLevel(_maxlevel)).size() != 1 ) {
        RAVELOG_WARN("more than 1 root node\n");
        return false;
    }

    dReal fLevelBound = _fMaxLevelBound;
    std::vector<SimpleNodePtr> vAccumNodes;
    vAccumNodes.reserve(_numnodes);
    size_t nallchildren = 0;
    size_t numnodes = 0;
    for(int currentlevel = _maxlevel; currentlevel >= _minlevel; --currentlevel, fLevelBound *= _fBaseInv ) {
        int enclevel = _EncodeLevel(currentlevel);
        if( enclevel >= (int)_vsetLevelNodes.size() ) {
            continue;
        }

        const std::set<SimpleNodePtr>& setLevelRawChildren = _vsetLevelNodes.at(enclevel);
        for(const SimpleNodePtr& node : setLevelRawChildren) {
            for(const SimpleNodePtr& child : node->_vchildren) {
                const dReal curdist = _ComputeDistance(node, child);
                if( curdist > fLevelBound + g_fEpsilonLinear ) {
#ifdef _DEBUG
                    RAVELOG_WARN_FORMAT("invalid parent child nodes %d, %d at level %d (%f), dist=%f",
                                        node->id % child->id % currentlevel % fLevelBound % curdist);
#else
                    RAVELOG_WARN_FORMAT("invalid parent child nodes at level %d (%f), dist=%f", currentlevel % fLevelBound%curdist);
#endif
                    return false;
                }
            }
            nallchildren += node->_vchildren.size();
            if( !node->_hasselfchild ) {
                vAccumNodes.push_back(node);
            }

            if( currentlevel < _maxlevel ) {
                // find its parents
                int nfound = 0;
                for(const SimpleNodePtr& parent : _vsetLevelNodes.at(_EncodeLevel(currentlevel+1))) {
                    if( find(parent->_vchildren.begin(), parent->_vchildren.end(), node) != parent->_vchildren.end() ) {
                        ++nfound;
                    }
                }
                BOOST_ASSERT(nfound==1);
            }
        }

        numnodes += setLevelRawChildren.size();

        const size_t nNodes = vAccumNodes.size();
        for(size_t i = 0; i < nNodes; ++i) {
            for(size_t j = i+1; j < nNodes; ++j) {
                const dReal curdist = _ComputeDistance(vAccumNodes[i], vAccumNodes[j]);
                if( curdist <= fLevelBound ) {
#ifdef _DEBUG
                    RAVELOG_WARN_FORMAT("invalid sibling nodes %d, %d  at level %d (%f), dist=%f", vAccumNodes[i]->id%vAccumNodes[j]->id%currentlevel%fLevelBound%curdist);
#else
                    RAVELOG_WARN_FORMAT("invalid sibling nodes %d, %d  at level %d (%f), dist=%f", i%j%currentlevel%fLevelBound%curdist);
#endif
                    return false;
                }
            }
        }
    }

    if( _numnodes != (int)numnodes ) {
        RAVELOG_WARN_FORMAT("num predicted nodes (%d) does not match computed nodes (%d)", _numnodes%numnodes);
        return false;
    }
    if( _numnodes != (int)nallchildren+1 ) {
        RAVELOG_WARN_FORMAT("num predicted nodes (%d) does not match computed nodes from children (%d)", _numnodes%(nallchildren+1));
        return false;
    }

    return true;
}

void SpatialTree::DumpTree(std::ostream& o) const
{
    o << _numnodes << endl;
    std::vector<SimpleNodePtr> vnodes;
    this->GetNodesVector(vnodes);
    for(const SimpleNodePtr& node : vnodes) {
        for(int i = 0; i < _dof; ++i) {
            o << node->q[i] << ",";
        }
        auto itnode = find(begin(vnodes), end(vnodes), node->rrtparent);
        if( itnode == end(vnodes) ) {
            o << "-1" << endl;
        }
        else {
            o << (size_t)(itnode - begin(vnodes)) << endl;
        }
    }
}

/// \brief given random index 0 <= inode < _numnodes, return a node. If tree changes, indices might change
SimpleNodePtr SpatialTree::GetNodeFromIndex(size_t inode) const
{
    if( (int)inode >= _numnodes ) {
        return SimpleNodePtr();
    }
    for(const std::set<SimpleNodePtr>& sLevelNodes : _vsetLevelNodes) {
        if( inode < sLevelNodes.size() ) {
            typename std::set<SimpleNodePtr>::iterator itchild = sLevelNodes.begin();
            advance(itchild, inode);
            return *itchild;
        }
        else {
            inode -= sLevelNodes.size();
        }
    }
    return SimpleNodePtr();
}

void SpatialTree::GetNodesVector(std::vector<SimpleNodePtr>& vnodes) const
{
    vnodes.clear();
    if( (int)vnodes.capacity() < _numnodes ) {
        vnodes.reserve(_numnodes);
    }
    for(const std::set<SimpleNodePtr>& sLevelNodes : _vsetLevelNodes) {
        vnodes.insert(vnodes.end(), sLevelNodes.begin(), sLevelNodes.end());
    }
}

int SpatialTree::GetNewStaticId() {
    static int s_id = 0;
    int retid = s_id++;
    return retid;
}

SimpleNodePtr SpatialTree::_CreateNode(SimpleNodePtr rrtparent, const std::vector<dReal>& config, uint32_t userdata)
{
    // allocate memory for the structur and the internal state vectors
    void* pmemory = _pNodesPool->malloc();
    SimpleNodePtr node = new (pmemory) SimpleNode(rrtparent, config);
    node->_userdata = userdata;
#ifdef _DEBUG
    node->id = GetNewStaticId();
#endif
    return node;
}

SimpleNodePtr SpatialTree::_CloneNode(SimpleNodePtr refnode)
{
    // allocate memory for the structur and the internal state vectors
    void* pmemory = _pNodesPool->malloc();
    SimpleNodePtr node = new (pmemory) SimpleNode(refnode->rrtparent, refnode->q, _dof);
    node->_userdata = refnode->_userdata;
#ifdef _DEBUG
    node->id = GetNewStaticId();
#endif
    return node;
}

void SpatialTree::_DeleteNode(SimpleNodePtr p)
{
    if( !!p ) {
        p->~SimpleNode();
        _pNodesPool->free(p);
    }
}

int SpatialTree::_EncodeLevel(int level) const {
    level <<= 1;
    return (level > 0) ? (level + 1) : (-level);
}

std::pair<SimpleNodePtr, dReal> SpatialTree::_FindNearestNode(const std::vector<dReal>& vquerystate) const
{
    std::pair<SimpleNodePtr, dReal> bestnode;
    bestnode.first = NULL;
    bestnode.second = std::numeric_limits<dReal>::infinity();
    if( _numnodes == 0 ) {
        return bestnode;
    }
    OPENRAVE_ASSERT_OP((int)vquerystate.size(),==,_dof);

    int currentlevel = _maxlevel; // where the root node is
    // traverse all levels gathering up the children at each level
    dReal fLevelBound = _fMaxLevelBound;
    _vCurrentLevelNodes.resize(1);
    _vCurrentLevelNodes[0].first = *_vsetLevelNodes.at(_EncodeLevel(_maxlevel)).begin();
    _vCurrentLevelNodes[0].second = _ComputeDistance(_vCurrentLevelNodes[0].first->q, vquerystate);
    if( _vCurrentLevelNodes[0].first->_usenn ) {
        bestnode = _vCurrentLevelNodes[0];
    }
    while(!_vCurrentLevelNodes.empty() ) {
        _vNextLevelNodes.clear();
        //RAVELOG_VERBOSE_FORMAT("level %d (%f) has %d nodes", currentlevel%fLevelBound%_vCurrentLevelNodes.size());
        dReal minchilddist=std::numeric_limits<dReal>::infinity();
        FOREACH(itcurrentnode, _vCurrentLevelNodes) {
            // only take the children whose distances are within the bound
            FOREACHC(itchild, itcurrentnode->first->_vchildren) {
                dReal curdist = _ComputeDistance((*itchild)->q, vquerystate);
                if( !bestnode.first || (curdist < bestnode.second && bestnode.first->_usenn)) {
                    bestnode = make_pair(*itchild, curdist);
                }
                _vNextLevelNodes.emplace_back(*itchild,  curdist);
                if( minchilddist > curdist ) {
                    minchilddist = curdist;
                }
            }
        }

        _vCurrentLevelNodes.clear();
        dReal ftestbound = minchilddist + fLevelBound;
        FOREACH(itnode, _vNextLevelNodes) {
            if( itnode->second < ftestbound ) {
                _vCurrentLevelNodes.push_back(*itnode);
            }
        }
        currentlevel -= 1;
        fLevelBound *= _fBaseInv;
    }
    //RAVELOG_VERBOSE_FORMAT("query went through %d levels", (_maxlevel-currentlevel));
    return bestnode;
}

SimpleNodePtr SpatialTree::_InsertNode(SimpleNodePtr parent, const vector<dReal>& config, uint32_t userdata)
{
    SimpleNodePtr newnode = _CreateNode(parent, config, userdata);
    if( _numnodes == 0 ) {
        // no root
        _vsetLevelNodes.at(_EncodeLevel(_maxlevel)).insert(newnode); // add to the level
        newnode->_level = _maxlevel;
        _numnodes += 1;
    }
    else {
        _vCurrentLevelNodes.resize(1);
        _vCurrentLevelNodes[0].first = *_vsetLevelNodes.at(_EncodeLevel(_maxlevel)).begin();
        _vCurrentLevelNodes[0].second = _ComputeDistance(_vCurrentLevelNodes[0].first->q, config);
        int nParentFound = _InsertRecursive(newnode, _vCurrentLevelNodes, _maxlevel, _fMaxLevelBound);
        if( nParentFound == 0 ) {
            // could possibly happen with circulr joints, still need to take a look at correct fix (see #323)
            std::stringstream ss; ss << std::setprecision(std::numeric_limits<dReal>::digits10+1);
            FOREACHC(it, config) {
                ss << *it << ",";
            }
            throw OPENRAVE_EXCEPTION_FORMAT("Could not insert config=[%s] inside the cover tree, perhaps cover tree _maxdistance=%f is not enough from the root", ss.str()%_maxdistance, ORE_Assert);
        }
        if( nParentFound < 0 ) {
            return SimpleNodePtr();
        }
    }
    //BOOST_ASSERT(Validate());
    return newnode;
}

int SpatialTree::_InsertRecursive(SimpleNodePtr nodein,
                     const std::vector< std::pair<SimpleNodePtr, dReal> >& vCurrentLevelNodes,
                     int currentlevel,
                     dReal fLevelBound)
{
#ifdef _DEBUG
    // copy for debugging
    std::vector< std::pair<SimpleNodePtr, dReal> > vLocalLevelNodes = vCurrentLevelNodes;
#endif
    dReal closestDist=std::numeric_limits<dReal>::infinity();
    SimpleNodePtr closestNodeInRange=NULL; /// one of the nodes in vCurrentLevelNodes such that its distance to nodein is <= fLevelBound
    int enclevel = _EncodeLevel(currentlevel);
    if( enclevel < (int)_vsetLevelNodes.size() ) {
        // build the level below
        _vNextLevelNodes.clear(); // for currentlevel-1
        for(const std::pair<SimpleNodePtr, dReal>& currnode : vCurrentLevelNodes) {
            if( currnode.second <= fLevelBound ) {
                if( !closestNodeInRange ) {
                    closestNodeInRange = currnode.first;
                    closestDist = currnode.second;
                }
                else {
                    if(  currnode.second < closestDist-g_fEpsilonLinear ) {
                        closestNodeInRange = currnode.first;
                        closestDist = currnode.second;
                    }
                    // if distances are close, get the node on the lowest level...
                    else if( currnode.second < closestDist+_mindistance && currnode.first->_level < closestNodeInRange->_level ) {
                        closestNodeInRange = currnode.first;
                        closestDist = currnode.second;
                    }
                }
                if ( (closestDist <= _mindistance) ) {
                    // pretty close, so return as if node was added
                    return -1;
                }
            }
            if( currnode.second <= fLevelBound*_fBaseChildMult ) {
                // node is part of all sets below its level
                _vNextLevelNodes.push_back(currnode);
            }
            // only take the children whose distances are within the bound
            if( currnode.first->_level == currentlevel ) {
                FOREACHC(itchild, currnode.first->_vchildren) {
                    dReal curdist = _ComputeDistance(nodein, *itchild);
                    if( curdist <= fLevelBound*_fBaseChildMult ) {
                        _vNextLevelNodes.emplace_back(*itchild,  curdist);
                    }
                }
            }
        }

        if( !_vNextLevelNodes.empty() ) {
            _vCurrentLevelNodes.swap(_vNextLevelNodes); // invalidates vCurrentLevelNodes
            // note that after _Insert call, _vCurrentLevelNodes could be complete lost/reset
            int nParentFound = _InsertRecursive(nodein, _vCurrentLevelNodes, currentlevel-1, fLevelBound*_fBaseInv);
            if( nParentFound != 0 ) {
                return nParentFound;
            }
        }
    }
    else {
        for(const std::pair<SimpleNodePtr, dReal>& currnode : vCurrentLevelNodes) {
            if( currnode.second <= fLevelBound ) {
                if( !closestNodeInRange ) {
                    closestNodeInRange = currnode.first;
                    closestDist = currnode.second;
                }
                else {
                    if(  currnode.second < closestDist-g_fEpsilonLinear ) {
                        closestNodeInRange = currnode.first;
                        closestDist = currnode.second;
                    }
                    // if distances are close, get the node on the lowest level...
                    else if( currnode.second < closestDist+_mindistance && currnode.first->_level < closestNodeInRange->_level ) {
                        closestNodeInRange = currnode.first;
                        closestDist = currnode.second;
                    }
                }
                if ( (closestDist < _mindistance) ) {
                    // pretty close, so return as if node was added
                    return -1;
                }
            }
        }
    }

    if( !closestNodeInRange ) {
        return 0;
    }

    _InsertDirectly(nodein, closestNodeInRange, closestDist, currentlevel-1, fLevelBound*_fBaseInv);
    _numnodes += 1;
    return 1;
}

bool SpatialTree::_InsertDirectly(SimpleNodePtr nodein, SimpleNodePtr parentnode, dReal parentdist, int maxinsertlevel, dReal fInsertLevelBound)
{
    int insertlevel = maxinsertlevel;
    if( parentdist <= _mindistance ) {
        // pretty close, so notify parent that there's a similar child already underneath it
        if( parentnode->_hasselfchild ) {
            // already has a similar child, so go one level below...?
            FOREACH(itchild, parentnode->_vchildren) {
                dReal childdist = _ComputeDistance(nodein, *itchild);
                if( childdist <= _mindistance ) {
                    return _InsertDirectly(nodein, *itchild, childdist, maxinsertlevel-1, fInsertLevelBound*_fBaseInv);
                }
            }
            RAVELOG_WARN("inconsistent node found\n");
            return false;
        }
    }
    else {
        // depending on parentdist, might have to insert at a lower level in order to keep the sibling invariant
        dReal fChildLevelBound = fInsertLevelBound;
        while(parentdist < fChildLevelBound) {
            fChildLevelBound *= _fBaseInv;
            insertlevel--;
        }
    }

    // have to add at insertlevel. If currentNodeInRange->_level is > insertlevel+1, will have to clone it. note that it will still represent the same RRT node with same rrtparent
    while( parentnode->_level > insertlevel+1 ) {
        SimpleNodePtr clonenode = _CloneNode(parentnode);
        clonenode->_level = parentnode->_level-1;
        parentnode->_vchildren.push_back(clonenode);
        parentnode->_hasselfchild = 1;
        int encclonelevel = _EncodeLevel(clonenode->_level);
        if( encclonelevel >= (int)_vsetLevelNodes.size() ) {
            _vsetLevelNodes.resize(encclonelevel+1);
        }
        _vsetLevelNodes.at(encclonelevel).insert(clonenode);
        _numnodes +=1;
        parentnode = clonenode;
    }

    if( parentdist <= _mindistance ) {
        parentnode->_hasselfchild = 1;
    }
    nodein->_level = insertlevel;
    int enclevel2 = _EncodeLevel(nodein->_level);
    if( enclevel2 >= (int)_vsetLevelNodes.size() ) {
        _vsetLevelNodes.resize(enclevel2+1);
    }
    _vsetLevelNodes.at(enclevel2).insert(nodein);
    parentnode->_vchildren.push_back(nodein);

    if( _minlevel > nodein->_level ) {
        _minlevel = nodein->_level;
    }
    return true;
}

bool SpatialTree::_RemoveNode(SimpleNodePtr removenode)
{
    if( _numnodes == 0 ) {
        return false;
    }

    SimpleNodePtr proot = *_vsetLevelNodes.at(_EncodeLevel(_maxlevel)).begin();
    if( _numnodes == 1 && removenode == proot ) {
        Reset();
        return true;
    }

    if( _maxlevel-_minlevel >= (int)_vvCacheNodes.size() ) {
        _vvCacheNodes.resize(_maxlevel-_minlevel+1);
    }
    for(std::vector<SimpleNodePtr>& vCacheNodes : _vvCacheNodes) {
        vCacheNodes.clear();
    }
    _vvCacheNodes.at(0).push_back(proot);
    bool bRemoved = _Remove(removenode, _vvCacheNodes, _maxlevel, _fMaxLevelBound);
    if( bRemoved ) {
        _DeleteNode(removenode);
    }
    if( removenode == proot ) {
        BOOST_ASSERT(_vvCacheNodes.at(0).size()==2); // instead of root, another node should have been added
        BOOST_ASSERT(_vsetLevelNodes.at(_EncodeLevel(_maxlevel)).size()==1);
        //_vsetLevelNodes.at(_EncodeLevel(_maxlevel)).clear();
        _vsetLevelNodes.at(_EncodeLevel(_maxlevel)).erase(proot);
        bRemoved = true;
        _numnodes--;
    }
    return bRemoved;
}

bool SpatialTree::_Remove(SimpleNodePtr removenode, std::vector< std::vector<SimpleNodePtr> >& vvCoverSetNodes, int currentlevel, dReal fLevelBound)
{
    const int enclevel = _EncodeLevel(currentlevel);
    if( enclevel >= (int)_vsetLevelNodes.size() ) {
        return false;
    }

    // build the level below
    std::set<SimpleNodePtr>& setLevelRawChildren = _vsetLevelNodes.at(enclevel);
    int coverindex = _maxlevel-(currentlevel-1);
    if( coverindex >= (int)vvCoverSetNodes.size() ) {
        vvCoverSetNodes.resize(coverindex+(_maxlevel-_minlevel)+1);
    }
    std::vector<SimpleNodePtr>& vNextLevelNodes = vvCoverSetNodes[coverindex];
    vNextLevelNodes.clear();

    bool bfound = false;
    FOREACH(itcurrentnode, vvCoverSetNodes.at(coverindex-1)) {
        // only take the children whose distances are within the bound
        if( setLevelRawChildren.find(*itcurrentnode) != setLevelRawChildren.end() ) {
            typename std::vector<SimpleNodePtr>::iterator itchild = (*itcurrentnode)->_vchildren.begin();
            while(itchild != (*itcurrentnode)->_vchildren.end() ) {
                dReal curdist = _ComputeDistance(removenode, *itchild);
                if( *itchild == removenode ) {
                    //vNextLevelNodes.clear();
                    vNextLevelNodes.push_back(*itchild);
                    itchild = (*itcurrentnode)->_vchildren.erase(itchild);
                    if( (*itcurrentnode)->_hasselfchild && _ComputeDistance(*itcurrentnode, *itchild) <= _mindistance) {
                        (*itcurrentnode)->_hasselfchild = 0;
                    }
                    bfound = true;
                    //break;
                }
                else {
                    if( curdist <= fLevelBound*_fBaseChildMult ) {
                        vNextLevelNodes.push_back(*itchild);
                    }
                    ++itchild;
                }
            }
//                if( bfound ) {
//                    break;
//                }
        }
    }

    bool bRemoved = _Remove(removenode, vvCoverSetNodes, currentlevel-1, fLevelBound*_fBaseInv);

    if( !bRemoved && removenode->_level == currentlevel && find(vvCoverSetNodes.at(coverindex-1).begin(), vvCoverSetNodes.at(coverindex-1).end(), removenode) != vvCoverSetNodes.at(coverindex-1).end() ) {
        // for each child, find a more suitable parent
        FOREACH(itchild, removenode->_vchildren) {
            int parentlevel = currentlevel;
            dReal fParentLevelBound = fLevelBound;
            dReal closestdist=0;
            SimpleNodePtr closestNode = NULL;
            //int maxaddlevel = currentlevel-1;
            while(parentlevel <= _maxlevel  ) {
                FOREACHC(itnode, vvCoverSetNodes.at(_maxlevel-parentlevel)) {
                    if( *itnode == removenode ) {
                        continue;
                    }
                    dReal curdist = _ComputeDistance(*itchild, *itnode);
                    if( curdist < fParentLevelBound ) {
                        if( !closestNode || curdist < closestdist ) {
                            closestdist = curdist;
                            closestNode = *itnode;
                        }
                    }
                }
                if( !!closestNode ) {
                    SimpleNodePtr nodechild = *itchild;
                    while( nodechild->_level < closestNode->_level-1 ) {
                        SimpleNodePtr clonenode = _CloneNode(nodechild);
                        clonenode->_level = nodechild->_level+1;
                        clonenode->_vchildren.push_back(nodechild);
                        clonenode->_hasselfchild = 1;
                        int encclonelevel = _EncodeLevel(clonenode->_level);
                        if( encclonelevel >= (int)_vsetLevelNodes.size() ) {
                            _vsetLevelNodes.resize(encclonelevel+1);
                        }
                        _vsetLevelNodes.at(encclonelevel).insert(clonenode);
                        _numnodes +=1;
                        vvCoverSetNodes.at(_maxlevel-clonenode->_level).push_back(clonenode);
                        nodechild = clonenode;
                    }

                    if( closestdist <= _mindistance ) {
                        closestNode->_hasselfchild = 1;
                    }

                    closestNode->_vchildren.push_back(nodechild);
                    break;
                }

                // try a higher level
                parentlevel += 1;
                fParentLevelBound *= _base;
            }
            if( !closestNode ) {
                BOOST_ASSERT(parentlevel>_maxlevel);
                // occurs when root node is being removed and new children have no where to go?
                _vsetLevelNodes.at(_EncodeLevel(_maxlevel)).insert(*itchild);
                vvCoverSetNodes.at(0).push_back(*itchild);
            }
        }
        // remove the node
        size_t erased = setLevelRawChildren.erase(removenode);
        BOOST_ASSERT(erased==1);
        bRemoved = true;
        _numnodes--;
    }
    return bRemoved;
}