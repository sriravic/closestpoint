#include <closestpointquery.h>
#include <geometry.h>
#include <thread>
#include <util.h>

namespace
{
    static const int NUM_QUERY_THREADS = 8;

    static const char*
    getMethodString(QueryMethod m)
    {
        if (m == QueryMethod::QUERY_BRUTE_FORCE) return "brute force";
        else return "acceleration structure";
    }
}

template<typename DataType, typename IndexType>
bool
ClosestPointQuery<DataType, IndexType>::init(const std::string& meshname,
    QueryMethod m)
{
    auto ret = myMesh.open(meshname);
    if (ret)
    {
        myMesh.printStats();
        if (m == QueryMethod::QUERY_OPTIMIZED)
        {
            std::cout << "Building KdTree....." << std::endl;
            Timer t("Kd Tree Build Timer");

            t.start();
            myAccel.build(myMesh);
            t.stop();
            t.printElapsed();
        }
        myInitialized = true;
    }
    return ret;
}

template<typename DataType, typename IndexType>
void
ClosestPointQuery<DataType, IndexType>::query(const InputType& inputs,
    QueryMethod m, OutputType& outputs, DataType searchRadius) const noexcept
{
    if (!myInitialized)
    {
        std::cerr << "Query Manager is not initialized."
                  << "Call init() first" << std::endl;
        return;
    }

    std::cout << "Running " << getMethodString(m) << " method for "
        << inputs.size() << " query points" << std::endl;

    switch (m)
    {
        case QueryMethod::QUERY_BRUTE_FORCE:
        {
            Timer t("Brute Force timer");
            
            t.start();
            bruteforce(inputs, outputs, searchRadius);
            t.stop();
            
            t.printElapsed();
            break;
        }
        case QueryMethod::QUERY_OPTIMIZED:
        {
            Timer t("Kd Tree timer");

            t.start();
            queryKdTree(inputs, outputs, searchRadius);
            t.stop();

            t.printElapsed();
            break;
        }
    }
}

template<typename DataType, typename IndexType>
void
ClosestPointQuery<DataType, IndexType>::bruteforce(const InputType& inputs,
    OutputType& outputs, DataType searchRadius) const noexcept
{
    const auto& mesh = myMesh;
    outputs.resize(inputs.size());
    for (size_t pidx = 0, np = inputs.size(); pidx < np; pidx++)
    {
        auto&& output = outputs[pidx];
        auto&& input = inputs[pidx];
        
        // multi threaded search
        // assuming #searchPts << #triangles
        QueryResultType threadAns[NUM_QUERY_THREADS];
        std::vector<std::thread> queryThreads;
        auto&& nqt = NUM_QUERY_THREADS;

        for (int t = 0; t < NUM_QUERY_THREADS; t++)
        {
            queryThreads.emplace_back(
                std::thread([&mesh, &threadAns, &input, t, searchRadius, nqt]()
                {
                    // Each thread searches for in a separate range of ids
                    auto&& numPrims = mesh.getNumPrimitives();
                    auto&& worksize = numPrims / nqt;
                    auto&& startIdx = worksize * t +
                        ((numPrims % nqt == 0) ? 0 : 1);
                    auto&& endIdx    = std::min(numPrims, startIdx + worksize);
                    auto&& localResult = threadAns[t];

                    for (IndexType idx = startIdx; idx < endIdx; idx++)
                    {
                        QueryResultType temp;
                        auto&& prim = mesh.getPrimitive(idx);
                        computeTriangleClosestPoint<DataType, IndexType>(
                            prim, input, idx, temp);

                        if (temp <= localResult)
                        {
                            if (temp < localResult)
                                localResult.clear();
                            localResult.appendResults(temp);
                        }
                    }
                }));
        }

        // wait for threads to finish searching
        for (int t = 0; t < NUM_QUERY_THREADS; t++)
            queryThreads[t].join();

        // do a linear search reduction over the results
        output = QueryResultType(searchRadius * searchRadius);
        for (int i = 0; i < NUM_QUERY_THREADS; i++)
        {
            if (threadAns[i] <= output)
            {
                if (threadAns[i] < output)
                    output.clear();
                output.appendResults(threadAns[i]);
            }
        }
    }
}

template<typename DataType, typename IndexType>
void
ClosestPointQuery<DataType, IndexType>::queryKdTree(const InputType& input,
    OutputType& output, DataType searchRadius) const noexcept
{
    // alllocate space for results
    const auto& accel   = myAccel;
    auto&& nqt          = NUM_QUERY_THREADS;
    
    output.resize(input.size());
    std::vector<std::thread> queryThreads;
    
    for (int t = 0; t < nqt; t++)
        queryThreads.emplace_back(std::thread(
            [&accel, &input, &output, searchRadius, t, nqt]()
            {
                // compute ptid to compute results for
                auto&& numinputs = input.size();
                auto&& worksize  = numinputs / nqt +
                    ((numinputs % nqt == 0) ? 0 : 1);
                auto&& startIdx  = worksize * t;
                auto&& endIdx    = std::min(startIdx + worksize, numinputs);

                for (auto idx = startIdx; idx < endIdx; idx++)
                {
                    auto&& result = output[idx];
                    auto&& inputpt = input[idx];
                    accel.query(inputpt, searchRadius, result);
                }
            }
    ));

    for (int t = 0; t < NUM_QUERY_THREADS; t++)
        queryThreads[t].join();
}

template class ClosestPointQuery<float, uint32_t>;
template class ClosestPointQuery<double, uint32_t>;
