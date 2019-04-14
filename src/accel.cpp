#include <accel.h>
#include <cmath>
#include <stack>
#include <tuple>

namespace
{
    // Convenient indexing tags for left, right or plane 
    // used in SAH construction
    constexpr static uint8_t TAG_N_LEFT  = 0;
    constexpr static uint8_t TAG_N_PLANE = 1;
    constexpr static uint8_t TAG_N_RIGHT = 2;

    // tags used for filtering primitives
    constexpr static uint8_t TAG_LEFT    = 0;
    constexpr static uint8_t TAG_RIGHT   = 1;
    constexpr static uint8_t TAG_BOTH    = 2;

    template<typename T>
    static T
    log2ToType(float val)
    {
        return static_cast<T>(std::floor(std::log2(val)));
    }

#ifdef _DEBUG
    constexpr static uint8_t assertLvl = 1;
#else
    constexpr static uint8_t assertLvl = 0;
#endif
}

template<typename DataType, typename IndexType>
void
KdTree<DataType, IndexType>::build(MeshType& mesh)
{
    // store a reference for intersection purposes to query vertex data
    myMeshPtr = &mesh;
    const auto numPrimitives = mesh.getNumPrimitives();

    // store for usage while querying
    mySceneBounds = mesh.getBounds();

    // compute max depth
    // similar to pbrt's style, but tunable.
    myMaxDepth = 4 + log2ToType<IndexType>(static_cast<float>(numPrimitives));

    // create bounding boxes for all individual triangles
    EventListType root;
    root.myBounds = mySceneBounds;
    root.myPrimIds.resize(numPrimitives);
    
    myTempTags.resize(numPrimitives);
    myOriginalBounds.resize(numPrimitives);
    for (IndexType i = 0; i < numPrimitives; i++)
    {
        root.myPrimIds[i]   = i;
        myOriginalBounds[i] = mesh.getBounds(i);
    }

    // Each primitive can create start & end or planar events
    // Each triangle has 3 vertices, so at max 6 events
    auto& ebuffer = root.myEvents;
    ebuffer.reserve(numPrimitives * 6);
    for(uint8_t dim = 0; dim < 3; dim++)
        for (IndexType i = 0; i < numPrimitives; i++)
        {
            auto&& bbox = myOriginalBounds[i];
            if (bbox.bmin[dim] == bbox.bmax[dim])
                ebuffer.emplace_back(bbox.bmin[dim], i, dim, EVENT_PLANAR);
            else
            {
                ebuffer.emplace_back(bbox.bmin[dim], i, dim, EVENT_START);
                ebuffer.emplace_back(bbox.bmax[dim], i, dim, EVENT_END);
            }
        }
    
    // allocate buffer storage
    size_t allocEvtsSize        = ebuffer.size();
    size_t allocIdSize          = root.myPrimIds.size();
    size_t allocBothEvtsSize    = static_cast<size_t>(std::sqrt(allocEvtsSize));
    size_t allocBothIdsSize     = static_cast<size_t>(std::sqrt(allocIdSize));
    myLOnlyBuffer.reserve(allocEvtsSize, allocIdSize);
    myROnlyBuffer.reserve(allocEvtsSize, allocIdSize);
    myLBothBuffer.reserve(allocBothEvtsSize, allocBothIdsSize);
    myRBothBuffer.reserve(allocBothEvtsSize, allocBothIdsSize);

    // reserve up space for nodes = 2^d - 1
    auto reserveSize = (1 << myMaxDepth) - 1;
    myNodes.reserve(reserveSize);

    // Now that we have all events for all triangles, we can sort
    // and construct the tree
    sortEventList(root);
    recursiveBuild(0, root);
}

template<typename DataType, typename IndexType>
void
KdTree<DataType, IndexType>::query(const VectorType& input,
    DataType searchRadius, QueryResultType& output) const noexcept
{
    using NodePairType  = std::pair<IndexType, BoxType>;
    using StackType     = std::stack<NodePairType>;
    
    StackType todo;
    DataType minRadius2 = searchRadius * searchRadius; // squared distance
    output = QueryResultType(minRadius2);

    todo.push(std::make_pair(0, mySceneBounds));
    while (!todo.empty())
    {
        NodePairType nodePair       = todo.top(); todo.pop();
        const KdTreeNodeType& node  = myNodes[nodePair.first];
        const BoxType& nodeBounds   = nodePair.second;
        if (squaredClosestDistance<DataType>(nodeBounds, input) > minRadius2)
            continue;
        else
        {
            // check if leaf
            if (node.isleaf())
            {
                for (IndexType prim = 0; prim < node.getNumPrims(); prim++)
                {
                    QueryResultType temp;
                    auto primIdx = node.getPrimId(prim);
                    auto&& tri = myMeshPtr->getPrimitive(primIdx);
                    computeTriangleClosestPoint<DataType, IndexType>(
                        tri, input, primIdx, temp);

                    if (temp <= output)
                    {
                        // update search radius for next round
                        minRadius2 = temp.myClosestDistance;
                        if (temp < output)
                            output.clear();
                        output.appendResults(temp);
                    }
                }
            }
            else
            {
                // internal node. check for nearest node first and push it last
                // so that it can be popped later.
                uint8_t splitDim = node.getSplitDim();
                BoxType lbounds, rbounds;
                nodeBounds.split(node.getSplitPos(), splitDim, lbounds, rbounds);
                if (input[splitDim] <= node.getSplitPos())
                {
                    todo.push(std::make_pair(node.getChildOffset(), rbounds));
                    todo.push(std::make_pair(nodePair.first + 1, lbounds));
                }
                else
                {
                    todo.push(std::make_pair(nodePair.first + 1, lbounds));
                    todo.push(std::make_pair(node.getChildOffset(), rbounds));
                }
            }
        }
    }
}

template<typename DataType, typename IndexType>
void
KdTree<DataType, IndexType>::recursiveBuild(IndexType currentDepth,
    const EventListType& e)
{
    auto&& numPrims = static_cast<IndexType>(e.myPrimIds.size());
    if (currentDepth == myMaxDepth || numPrims < myMaxLeafPrims)
    {
        createLeaf(e);
        return;
    }
    
    // compute sah cost and determine if we should create a leaf/internal node
    CostType sahCost = findSplitPlane(e);
    if (terminate(sahCost, numPrims))
    {
        createLeaf(e);
        return;
    }
    else
    {
        // create internal node, compute  left and right lists and recurse
        EventListType left, right;
        auto&& id = createInternal(sahCost);
        computeChildSublists(e, sahCost, left, right);
        recursiveBuild(currentDepth + 1, left);
        
        // the next node inserted will be the right child of this node
        auto&& rchildLoc = static_cast<IndexType>(myNodes.size());
        myNodes[id].setRchildOffset(rchildLoc);
        recursiveBuild(currentDepth + 1, right);
    }
}

template<typename DataType, typename IndexType>
SplitCost<DataType>
KdTree<DataType, IndexType>::findSplitPlane(const EventListType& e) const noexcept
{
    // Have 3 types of costs for each dimension
    // row = dims [x,y,z];
    // col = type [-,|,+] (left, on, right)
    // numel = N in the paper    
    // default initialized with infinite cost
    auto&& events = e.myEvents;
    auto&& numel = static_cast<IndexType>(e.myPrimIds.size());
    CostType optimalCost = { std::numeric_limits<DataType>::max(),
                             DataType(0), SIDE_LEFT, 0 };
    
    IndexType N[3][3] = { {0, 0, numel},
                          {0, 0, numel},
                          {0, 0, numel} };
    size_t i = 0;
    while(i < events.size())
    {
        // initialize
        EventType current = events[i];
        IndexType p_start = 0, p_end = 0, p_plane = 0;

        /// Count the same type of events at the current split location
        while (i < events.size() &&
            events[i].splitDim == current.splitDim &&
            events[i].splitPos == current.splitPos &&
            events[i].eventMode == EVENT_END)
        {
            p_end++;
            i++;
        }

        while (i < events.size() &&
            events[i].splitDim == current.splitDim &&
            events[i].splitPos == current.splitPos &&
            events[i].eventMode == EVENT_PLANAR)
        {
            p_plane++;
            i++;
        }

        while (i < events.size() &&
            events[i].splitDim == current.splitDim &&
            events[i].splitPos == current.splitPos &&
            events[i].eventMode == EVENT_START)
        {
            p_start++;
            i++;
        }

        /// Update running counters for each dimension
        N[current.splitDim][TAG_N_PLANE]  = p_plane;
        N[current.splitDim][TAG_N_RIGHT] -= p_plane;
        N[current.splitDim][TAG_N_RIGHT] -= p_end;

        /// Compute the SAH cost for the current configuration
        CostType currCost = SAH(current, e.myBounds,
                                N[current.splitDim][TAG_N_LEFT],
                                N[current.splitDim][TAG_N_RIGHT],
                                N[current.splitDim][TAG_N_PLANE]);

        /// Update if minimal
        if (currCost < optimalCost)
            optimalCost = currCost;

        // update current counters
        N[current.splitDim][TAG_N_LEFT] += p_start;
        N[current.splitDim][TAG_N_LEFT] += p_plane;
        N[current.splitDim][TAG_N_PLANE] = 0;
    }

    // after traversing through all events
    // our condition is that we should have
    // visited all our primitives atleast one
    assert(N[0][0] == numel);
    assert(N[1][0] == numel);
    assert(N[2][0] == numel);

    return optimalCost;
}

template<typename DataType, typename IndexType>
void
KdTree<DataType, IndexType>::computeChildSublists(const EventListType& parent,
    const CostType& optimal, EventListType& left, EventListType& right) noexcept
{   
    /// first tag triangles as being left only/right only
    IndexType leftOnlyCnt = 0, rightOnlyCnt = 0, bothCnt = 0;
    tagPrimitives(parent, optimal, leftOnlyCnt, rightOnlyCnt, bothCnt);
    assert(parent.numPrims() == leftOnlyCnt + rightOnlyCnt + bothCnt);

    /// Compute split bounds
    parent.myBounds.split(optimal.splitPos, optimal.splitDim,
        left.myBounds, right.myBounds);
  
    /// Filter strictly left and right primitives
    myLOnlyBuffer.init(left.myBounds); myROnlyBuffer.init(right.myBounds);
    filterPrimitives(parent, myLOnlyBuffer, myROnlyBuffer);

    /// Split straddling prims and into left and right
    myLBothBuffer.init(left.myBounds); myRBothBuffer.init(right.myBounds);
    splitPrimitives(parent, myLBothBuffer, myRBothBuffer);

    /// Merge strictly and straddling prims on both sides
    mergeEventList(myLOnlyBuffer, myLBothBuffer, left);
    mergeEventList(myROnlyBuffer, myRBothBuffer, right);
}

template<typename DataType, typename IndexType>
void
KdTree<DataType, IndexType>::tagPrimitives(const EventListType& parent,
    const CostType& optimal, IndexType& leftOnlyCnt,
    IndexType& rightOnlyCnt, IndexType& bothCnt) noexcept
{
    // conservatively mark all triangles to be on both
    auto&& parentEvts = parent.myEvents;
    auto&& parentIds = parent.myPrimIds;
    bothCnt = static_cast<IndexType>(parentIds.size());
    for (auto&& ids : parentIds)
        myTempTags[ids] = TAG_BOTH;

    // tag left/right based on condition
    for (size_t e = 0, ne = parentEvts.size(); e < ne; e++)
    {
        auto&& triId = parentEvts[e].primIdx;
        if (parentEvts[e].splitDim == optimal.splitDim)
        {
            if (parentEvts[e].eventMode == EVENT_END &&
                parentEvts[e].splitPos <= optimal.splitPos)
            {
                // left only triangles 'end' before the split plane
                myTempTags[triId] = TAG_LEFT;
                leftOnlyCnt++;
                bothCnt--;
            }
            else if (parentEvts[e].eventMode == EVENT_START &&
                parentEvts[e].splitPos >= optimal.splitPos)
            {

                // right only triangles start after the split plane
                myTempTags[triId] = TAG_RIGHT;
                rightOnlyCnt++;
                bothCnt--;
            }
            else if (parentEvts[e].eventMode == EVENT_PLANAR)
            {
                // Planar triangles are categorized based on 
                // decision from SAH
                if (parentEvts[e].splitPos < optimal.splitPos ||
                    (parentEvts[e].splitPos == optimal.splitPos &&
                        optimal.sideMode == SIDE_LEFT))
                {
                    myTempTags[triId] = TAG_LEFT;
                    leftOnlyCnt++;
                    bothCnt--;
                }
                else if (parentEvts[e].splitPos > optimal.splitPos ||
                    (parentEvts[e].splitPos == optimal.splitPos &&
                        optimal.sideMode == SIDE_RIGHT))
                {
                    myTempTags[triId] = TAG_RIGHT;
                    rightOnlyCnt++;
                    bothCnt--;
                }
            }
        }
    }
}

template<typename DataType, typename IndexType>
void
KdTree<DataType, IndexType>::filterPrimitives(const EventListType& parent,
    EventListType& lchild, EventListType& rchild) noexcept
{
    /// reserve memory for optimization purposes
    auto&& leftOnly   = lchild.myEvents;
    auto&& lchildIds  = lchild.myPrimIds;
    auto&& rightOnly  = rchild.myEvents;
    auto&& rchildIds  = rchild.myPrimIds;
    auto&& parentEvts = parent.myEvents;
    auto&& parentIds  = parent.myPrimIds;
    
    /// filter events and ids appropriately
    for (size_t i = 0, ni = parentEvts.size(); i < ni; i++)
    {
        auto& evt = parentEvts[i];
        const auto& triId = evt.primIdx;
        if (myTempTags[triId] == TAG_LEFT)
            leftOnly.emplace_back(parentEvts[i]);
        else if (myTempTags[triId] == TAG_RIGHT)
            rightOnly.emplace_back(parentEvts[i]);
    }

    for (size_t i = 0, ni = parentIds.size(); i < ni; i++)
    {
        auto&& triId = parentIds[i];
        if (myTempTags[triId] == TAG_LEFT)
            lchildIds.emplace_back(triId);
        else if (myTempTags[triId] == TAG_RIGHT)
            rchildIds.emplace_back(triId);
    }

    assert(lchild.isvalid(assertLvl));
    assert(rchild.isvalid(assertLvl));
}

template<typename DataType, typename IndexType>
void
KdTree<DataType, IndexType>::splitPrimitives(const EventListType& parent,
    EventListType& lchild, EventListType& rchild) noexcept
{
    auto&& splitLeft    = lchild.myEvents;
    auto&& splitRight   = rchild.myEvents;
    auto&& lIds         = lchild.myPrimIds;
    auto&& rIds         = rchild.myPrimIds;
    auto&& parentIds    = parent.myPrimIds;

    // compute the split event list
    for (auto&& id : parentIds)
    {
        if (myTempTags[id] == TAG_BOTH)
        {
            // split and create two events. No perfect splits!
            const BoxType& obounds = myOriginalBounds[id];
            
            // clip the bounds to the current left and right bounds
            BoxType lbox = lchild.myBounds.intersectOf(obounds);
            BoxType rbox = rchild.myBounds.intersectOf(obounds);

            auto createEvent = [=](const BoxType& box, auto&& e)
            {
                for (uint8_t dim = 0; dim < 3; dim++)
                {
                    if (box.bmin[dim] == box.bmax[dim])
                        e.emplace_back(box.bmin[dim], id, dim, EVENT_PLANAR);
                    else
                    {
                        e.emplace_back(box.bmin[dim], id, dim, EVENT_START);
                        e.emplace_back(box.bmax[dim], id, dim, EVENT_END);
                    }
                }
            };
            
            createEvent(lbox, splitLeft);
            createEvent(rbox, splitRight);
            
            // add ids as well
            lIds.emplace_back(id);
            rIds.emplace_back(id);
        }
    }

    // sort event lists
    // ids dont have to be sorted as we strictly dont need a one-one mapping
    sortEventList(lchild);
    sortEventList(rchild);

    assert(lchild.isvalid(assertLvl));
    assert(rchild.isvalid(assertLvl));
}

template<typename DataType, typename IndexType>
void
KdTree<DataType, IndexType>::sortEventList(EventListType& e) noexcept
{
    std::sort(e.myEvents.begin(), e.myEvents.end(),
        [](const EventType& e1, const EventType& e2) {
            return e1 < e2;
        });
}

template<typename DataType, typename IndexType>
void
KdTree<DataType, IndexType>::mergeEventList(const EventListType& e1,
    const EventListType& e2, EventListType& result) noexcept
{
    auto&& resultIds    = result.myPrimIds;
    auto&& results      = result.myEvents;
    auto&& lid1         = e1.myPrimIds;
    auto&& lid2         = e2.myPrimIds;
    auto&& list1        = e1.myEvents;
    auto&& list2        = e2.myEvents;

    /// allocate space for merge
    resultIds.resize(lid1.size() + lid2.size());
    results.resize(list1.size() + list2.size());

    /// copy the ids first
    size_t ptr1 = 0, ptr2 = 0, index = 0;
    for (size_t i = 0, ni = lid1.size(); i < ni; i++)
        resultIds[index++] = lid1[i];
    for (size_t i = 0, ni = lid2.size(); i < ni; i++)
        resultIds[index++] = lid2[i];

    /// Merge the sorted lists
    index = 0;
    while (ptr1 < list1.size() && ptr2 < list2.size())
    {
        if (list1[ptr1] < list2[ptr2])
            results[index++] = list1[ptr1++];
        else
            results[index++] = list2[ptr2++];
    }

    /// Copy off the remaining ids
    while (ptr1 < list1.size())
        results[index++] = list1[ptr1++];
    while (ptr2 < list2.size())
        results[index++] = list2[ptr2++];

    assert(ptr1 == list1.size());
    assert(ptr2 == list2.size());
    assert(result.isvalid(assertLvl));
}

template<typename DataType, typename IndexType>
void
KdTree<DataType, IndexType>::createLeaf(const EventListType& e) noexcept
{
    auto&& primIds = e.myPrimIds;
    auto idx = myNodes.size();
    myNodes.emplace_back(KdTreeNode<DataType, IndexType>());
    myNodes[idx].initLeaf(primIds);
}

template<typename DataType, typename IndexType>
IndexType
KdTree<DataType, IndexType>::createInternal(const CostType& splitEvent) noexcept
{
    auto idx = myNodes.size();
    myNodes.emplace_back(KdTreeNode<DataType, IndexType>());
    myNodes[idx].initInternalNode(splitEvent.splitPos, splitEvent.splitDim);
    return static_cast<IndexType>(idx);
}

/// explicit template instantiations
template class KdTree<float, uint32_t>;
template class KdTree<double, uint32_t>;
