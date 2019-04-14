#ifndef __ACCEL_H__
#define __ACCEL_H__

#include <common.h>
#include <iostream>
#include <mesh.h>
#include <queryfunctions.h>
#include <map>

/// Events indicate where the triangles are with regards to split plane
enum EventMode : uint8_t
{
    EVENT_END = 0,
    EVENT_PLANAR,
    EVENT_START,
};

/// Tie breaking for triangles that lie on split plane
enum SideMode : uint8_t
{
    SIDE_LEFT = 0,
    SIDE_RIGHT
};

/// An event denotes a potential split location for a given primitive
/// along a dimension. Each primitive can generate at the max 2 events
/// along each dimension for a total of possibly 6 events
template<typename DataType, typename IndexType>
struct Event
{
    using EventType = Event<DataType, IndexType>;

    Event() = default;

    Event(DataType pos, IndexType id, uint8_t dim, EventMode mode)
        : splitPos(pos)
        , primIdx(id)
        , splitDim(dim)
        , eventMode(mode)
    {
    }
    
    EventType& operator= (const EventType& E)
    {
        splitPos = E.splitPos;
        primIdx = E.primIdx;
        splitDim = E.splitDim;
        eventMode = E.eventMode;
        return *this;
    }

    DataType    splitPos;
    IndexType   primIdx;
    uint8_t     splitDim;
    EventMode   eventMode;
    
    bool operator< (const EventType& e) const noexcept
    {
        if (splitPos == e.splitPos)
        {
            if (splitDim == e.splitDim)
                return eventMode < e.eventMode;
            else
                return splitDim < e.splitDim;
        }
        else
            return splitPos < e.splitPos;
    }

    bool operator<= (const EventType& e) const noexcept
    {
        if (splitPos == e.splitPos)
        {
            if (splitDim == e.splitDim)
                return eventMode <= e.eventMode;
            else
                return splitDim < e.splitDim;
        }
        else
            return splitPos < e.splitPos;
    }

    friend std::ostream& operator<< (std::ostream& out, const Event& E)
    {
        out << " [ " << E.splitPos << " , " << E.primIdx
            << " , " << int(E.splitDim) << " , " << int(E.eventMode) << " ] "
            << std::endl;
        return out;
    }
};

/// A convenient structure to hold a list of events belonging to a node
template<typename DataType, typename IndexType>
struct EventList : public NonCopyable
{
    using BoxType           = Box3<DataType>;
    using EventType         = Event<DataType, IndexType>;
    using EventListType     = std::vector<EventType>;
    using PrimIdListType    = std::vector<IndexType>;

    EventList() = default;
    EventList(const BoxType& bounds)
        : myBounds(bounds)
    {
    }

    void reserve(const size_t esize, const size_t isize)
    {
        myEvents.reserve(esize);
        myPrimIds.reserve(isize);
    }

    void init(const BoxType& bounds) noexcept
    {
        myBounds = bounds;
        myEvents.clear();
        myPrimIds.clear();
    }

    /// Each event should be contained within the bounding box
    /// ie. the location should be valid within the split dimension
    /// of the bounding box. Conveniently can be used to check
    /// and debug issues with the kd tree construction.
    /// In release builds this always returns true.
    bool isvalid(uint8_t
#ifdef _DEBUG
        debugLevel = 0
#endif
    )
    {

#ifdef _DEBUG

        for (auto&& e : myEvents)
            if (!(e.splitPos >= myBounds.bmin[e.splitDim] &&
                e.splitPos <= myBounds.bmax[e.splitDim]))
            {

                std::cerr << "Invalid event list" << std::endl;
                std::cerr << "Split Pos : " << e.splitPos << std::endl;
                std::cerr << "Split Dim : " << int(e.splitDim) << std::endl;
                std::cerr << "Prim Id   : " << e.primIdx << std::endl;
                std::cerr << "Bounds    : " << myBounds;

                return false;
            }

        if (debugLevel > 0)
        {
            /// sorted event list?
            if (myEvents.size())
            {
                for (size_t i = 0; i < myEvents.size() - 1; i++)
                {
                    if (!(myEvents[i] <= myEvents[i + 1]))
                        return false;
                }
            }

            /// a much more expensive debug check
            /// checks for valid prims ids also
            std::map<IndexType, IndexType> storedPrims;
            for (auto& i : myPrimIds)
                storedPrims.insert(std::make_pair(i, 0));

            /// check each event to see if its corresponding
            /// primitive is in the set
            for (auto&& e : myEvents)
            {
                auto find = storedPrims.find(e.primIdx);
                if (find == storedPrims.end())
                {
                    std::cerr << "Prim Id error : " << std::endl;
                    std::cerr << "Prim ID       : " << e.primIdx << std::endl;
                    return false;
                }
                else
                {
                    /// increment key
                    find->second++;
                }
            }

            /// now check if we have any ids that are not in events
            for(auto&& m : storedPrims)
                if (m.second == 0)
                {
                    std::cerr << "Invalid prim id exists in id list : "
                        << m.second << std::endl;
                    return false;
                }
        }
#endif
        return true;
    }

    void dump() const noexcept { std::cout << *this; }

    IndexType numPrims() const noexcept
    {
        return static_cast<IndexType>(myPrimIds.size());
    }

    BoxType getEventBounds() const noexcept
    {
        BoxType ret;
        auto&& bmin = ret.bmin;
        auto&& bmax = ret.bmax;
        for (auto&& e : myEvents)
        {
            bmin[e.splitDim] = std::min(bmin[e.splitDim], e.splitPos);
            bmax[e.splitDim] = std::max(bmin[e.splitDim], e.splitPos);
        }
        return ret;
    }

    friend std::ostream& operator<< (std::ostream& out, const EventList& E)
    {
        for (auto&& evt : E.myEvents)
            out << evt;
        return out;
    }

    // members
    EventListType   myEvents;
    PrimIdListType  myPrimIds;
    BoxType         myBounds;
};

/// Convenient structure to hold the SAH cost at a particular location
template<typename DataType>
struct SplitCost
{
    bool operator < (const SplitCost& C) const noexcept
    {
        return cost < C.cost;
    }

    DataType cost;
    DataType splitPos;
    SideMode sideMode;
    uint8_t  splitDim;
};

/// Linear KD tree node representation used for optimized retrieval purposes
template<typename DataType, typename IndexType>
struct KdTreeNode
{
    union
    {
        DataType    mySplitPos;
        IndexType   myOnePrimitive;
        IndexType*  myPrimitives;
    };
    union
    {
        IndexType  myFlags;       // 2bits [0,1,2] = split axis, [3] = leaf
        IndexType  myNumPrims;    // number of primitives for leaf node
        IndexType  myOtherChild;  // index into linear array for the second child
    };

    ~KdTreeNode()
    {
        // tricky to manage our own memory
        // if we are a leaf and have many prims
        if (isleaf() && getNumPrims() > 1)
            delete[] myPrimitives;
    }

    void initLeaf(const std::vector<IndexType>& primids)
    {
        // make sure to check for empty nodes
        auto nprims = primids.size();

        // store correct values
        myFlags = 3;
        myNumPrims |= (nprims << 2);

        if (nprims == 0)
            myOnePrimitive = IndexType(~0);    // sentinel
        else if (nprims == 1)
            myOnePrimitive = primids[0];
        else
        {
            myPrimitives = new IndexType[nprims];
            for (size_t i = 0; i < nprims; i++)
                myPrimitives[i] = primids[i];
        }
    }

    void initInternalNode(DataType splitPos, uint8_t splitDim)
    {
        mySplitPos = splitPos;
        myFlags = splitDim;
    }

    void setRchildOffset(IndexType offset)
    {
        assert(!isleaf());
        myOtherChild |= (offset << 2);
    }

    // convenient functions so as to not clobber up unions
    bool        isleaf() const { return (myFlags & 3) == 3; }
    IndexType   getNumPrims() const noexcept { return myNumPrims >> 2; }
    IndexType   getChildOffset() const noexcept { return myOtherChild >> 2; }
    
    IndexType   getPrimId(IndexType i) const noexcept
    {
        assert(isleaf());
        auto numPrims = getNumPrims();
        if(numPrims <= 1)
        {
            assert(i == 0);
            return myOnePrimitive; // returns sentinel if empty
        }
        else
        {
            assert(i < numPrims);
            return myPrimitives[i];
        }
    }
    
    DataType    getSplitPos() const { return mySplitPos; }
    uint8_t     getSplitDim() const noexcept
    {
        assert((myFlags & 3) <= 2); //[0,1,2] = [x,y,z]
        return myFlags & 3;
    }

};

// KdTree implementation from the paper
// Building SAH kd-trees in O(N log N) time
template<typename DataType, typename IndexType>
class KdTree : public NonCopyable
{
public:
    KdTree(DataType traversalCost = DataType(1),
        DataType intersectCost = DataType(80),
        IndexType maxLeafPrims = 4)
        : myTraversalCost(traversalCost)
        , myIntersectCost(intersectCost)
        , myMaxLeafPrims(maxLeafPrims)
        , myMeshPtr(nullptr)
    {
    }

    using MeshType          = Mesh<DataType, IndexType>;
    using ConstMeshPtr      = const MeshType*;
    using MeshPtr           = MeshType*;
    using VectorType        = Vector3<DataType>;
    using BoxType           = Box3<DataType>;
    using QueryResultType   = QueryResult<DataType, IndexType>;
    using EventType         = Event<DataType, IndexType>;
    using KdTreeNodeType    = KdTreeNode<DataType, IndexType>;
    using CostType          = SplitCost<DataType>;
    using BoxList           = std::vector<BoxType>;
    using KdTreeNodeList    = std::vector<KdTreeNodeType>;
    using ScratchList       = std::vector<uint8_t>;
    using EventListType     = EventList<DataType, IndexType>;
    
    void build(MeshType& mesh);
    void query(const VectorType& input, DataType searchRadius,
        QueryResultType& output) const noexcept;

private:

    /// recursive build
    void recursiveBuild(IndexType currentDepth, const EventListType& events);

    /// Convenient lambda scaling for empty space optimization
    static DataType
    lambdaP(const IndexType nleft, const IndexType nright)
    {
        if (nleft == 0 || nright == 0)
            return DataType(0.8);
        else return DataType(1);
    }

    /// Computes the SAH cost 
    DataType
    costFn(DataType leftProb, DataType rightProb,
        IndexType nleft, IndexType nright) const noexcept
    {
        return myTraversalCost + lambdaP(nleft, nright) * (
             myIntersectCost * (leftProb * nleft + rightProb * nright));
    }

    /// Computes the actual SAH cost given a split configuration
    /// for a given volume and split position with 
    /// count of primitives to the left, right and mid
    CostType
    SAH(const EventType& event, const BoxType& box,
        IndexType nleft, IndexType nright, IndexType nmid) const noexcept
    {
        BoxType left, right;
        box.split(event.splitPos, event.splitDim, left, right);

        // compute probabilities
        DataType denom = DataType(1) / box.area();
        DataType lprob = left.area() * denom;
        DataType rprob = right.area() * denom;
        
        assert(isValid(lprob));
        assert(isValid(rprob));

        DataType lcost = costFn(lprob, rprob, nleft + nmid, nright);
        DataType rcost = costFn(lprob, rprob, nleft, nright + nmid);
        if (lcost < rcost)
            return { lcost, event.splitPos, SIDE_LEFT, event.splitDim };
        else
            return { rcost, event.splitPos, SIDE_RIGHT, event.splitDim};
    }

    /// Quickly determine if we should just create a leaf
    bool
    terminate(const CostType& splitCost, IndexType primCnt) const noexcept
    {
        if (splitCost.cost > primCnt * myIntersectCost)
            return true;
        else return false;
    }

    /// Compute a split plane using SAH given a list of events
    CostType findSplitPlane(const EventListType& events) const noexcept;

    /// compute left and right split lists given a split location
    /// that was computed to have optimal SAH cost
    void computeChildSublists(const EventListType& parent, const CostType& opt,
        EventListType& lchild, EventListType& rchild) noexcept;

    /// Mark the primitives that belong entirely to the right,
    /// left or straddling given a split configuration
    void tagPrimitives(const EventListType& parent, const CostType& optimal,
        IndexType& leftOnlyCnt, IndexType& rightOnlyCnt, IndexType& bothCnt) noexcept;

    /// Given an event list, quickly filter the left only, right
    /// only events to respective lists
    void filterPrimitives(const EventListType& parent, EventListType& lchild,
        EventListType& rchild) noexcept;

    /// Compute splits for primitives that straddle the split plane
    /// both cnt indicates the number of prims of parent that straddles the
    /// split plane and is present in both left and right child
    void splitPrimitives(const EventListType& parent, EventListType& lchild,
        EventListType& rchild) noexcept;

    /// Convenient function to create a leaf node given an event list
    void createLeaf(const EventListType& events) noexcept;

    /// Create an internal node and provide the offset for the right child
    /// within the list of linear nodes. Returns the position in the linear
    /// list of nodes where the internal node was created
    IndexType createInternal(const CostType& optimal) noexcept;

    /// Convenient function to sort an event list
    /// based on Key(distance, splitDim, event)
    static void sortEventList(EventListType&) noexcept;

    /// assuming list1 and list2 are sorted already, merge them in linear time
    /// akin to merge sort step.
    static void mergeEventList(const EventListType& list1,
        const EventListType& list2, EventListType& result) noexcept;


    KdTreeNodeList  myNodes;            // linear nodes
    ScratchList     myTempTags;         // used to tag triangles while splitting
    BoxList         myOriginalBounds;   // we need access while splitting
    DataType        myTraversalCost;    // from raytracing literature
    DataType        myIntersectCost;    // from raytracing literature
    uint32_t        myMaxDepth;         // keep recursion from exploding
    IndexType       myMaxLeafPrims;     // a tunable parameter
    BoxType         mySceneBounds;      // store for querying
    ConstMeshPtr    myMeshPtr;

    // buffer storage for temporary memory
    EventListType   myLOnlyBuffer, myROnlyBuffer;
    EventListType   myLBothBuffer, myRBothBuffer;
};

using KdTreeF = KdTree<float, uint32_t>;
using KdTreeD = KdTree<double, uint32_t>;

#endif