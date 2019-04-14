#include <closestpointquery.h>
#include <iostream>
#include <random>
#include <sampling.h>
#include <util.h>

// External header file
#ifdef _WINDOWS
#pragma warning(push, 0)
#include <ProgramOptions.hxx>
#pragma warning(pop)
#else
#include <ProgramOptions.hxx>
#endif

namespace
{
    static const std::string theBruteMethod("brute");
    static const std::string theAccelMethod("accel");

    static void
    setupOptions(po::parser& parser)
    {
        parser["file"]
            .abbreviation('f')
            .type(po::string)
            .description("provide the input wavefront obj file. Only triangular meshes are supported for now.");

        parser["method"]
            .abbreviation('m')
            .type(po::string)
            .description("provide the method to employ for nearest point computation. Values can be 'brute' or 'accel'. 'brute' employs brute force computation whereas 'accel' employs a kd-tree to compute the nearest point for the mest");

        parser["point"]
            .abbreviation('p')
            .type(po::string)
            .description("provide the input 3d query point in [x,y,z] format");

        parser["pfile"]
            .abbreviation('i')
            .type(po::string)
            .description("provide an input newline separated file with each line containing query points in [x,y,z] format");

        parser["generate"]
            .abbreviation('g')
            .type(po::string)
            .description("generate #N random points within the bounding geometry of the mesh to test for nearest neighbour search. Available flags are 'sphere' and 'box'. Default is 'box'");

        parser["numpts"]
            .abbreviation('n')
            .type(po::i32)
            .description("number of random points to generate provided the the generate flag. Default is 10 pts");

        parser["radius"]
            .abbreviation('r')
            .type(po::f32)
            .description("provide a max search radius. All search is done using a squared version of this term. Default is infinity");

        parser["help"]
            .abbreviation('?')
            .description("print this help screen").callback([&]
        {
            std::cout << parser << '\n';
        });
    }

    static QueryMethod
    getMethod(const std::string& str)
    {
        if (str == theBruteMethod)
            return QueryMethod::QUERY_BRUTE_FORCE;
        else if (str == theAccelMethod)
            return QueryMethod::QUERY_OPTIMIZED;
        else
        {
            std::cerr << "Invalid method; using brute force" << std::endl;
            return QueryMethod::QUERY_BRUTE_FORCE;
        }
    }
}

int main(int argc, char** argv)
{
    // perform init operations
    po::parser parser;
    setupOptions(parser);
    parser.parse(argc, argv);

    if (argc == 1 || parser["help"].size())
    {
        std::cout << parser << std::endl;
        return -1;
    }

    auto&& file     = parser["file"];
    auto&& method   = parser["method"];
    auto&& point    = parser["point"];
    auto&& pfile    = parser["pfile"];
    auto&& sradius  = parser["radius"];
    auto&& generate = parser["generate"];
    auto&& ngen     = parser["numpts"];
    
    if (!file.size())
    {
        std::cout << "An input mesh file is required" << std::endl;
        std::cout << "See help for more details" << std::endl;
        return -1;
    }

    if (!method.size())
    {
        std::cout << "method argument 'brute'/'accel' is required" << std::endl;
        std::cout << "See help for more details" << std::endl;
        return -1;
    }
    
    using Type              = float;
    using VectorType        = Vector3<Type>;
    using OutputType        = QueryResult<Type, uint32_t>;
    using InputVectorType   = std::vector<VectorType>;
    using OutputVectorType  = std::vector<OutputType>;

    InputVectorType         inputs;
    OutputVectorType        outputs;
    QueryMethod             qmethod = getMethod(method.get().string);

    if (!(point.size() || generate.size()))
    {
        // generate random vectors
        std::cout << "No input query point[s] provided. Use generate flag"
            << " to generate random points inside bounding shape" << std::endl;
        return -1;
    }

    // Initialize and run queries
    ClosestPointQueryF queryManager;
    if (queryManager.init(file.get().string, qmethod))
    {
        // get a handle to our mesh
        auto&& mesh = queryManager.getMesh();

        // get all the input points
        if (point.size())
            inputs.emplace_back(getVector<Type>(point.get().string));
        else if (pfile.size())
            getVectors<Type>(pfile.get().string, inputs);
        else if (generate.size())
        {
            // get generate config
            int NUMGEN = 5;
            if (ngen.size())
                NUMGEN = ngen.get().i32;
            
            // setup generator
            std::default_random_engine rng(0xCAFEBA11);
            std::uniform_real_distribution<Type> dist;

            if (generate.get().string == std::string("sphere"))
            {
                auto&& bsphere = mesh.getBoundingSphere();
                for (auto i = 0; i < NUMGEN; i++)
                    inputs.emplace_back(sampleWithinSphere(bsphere,
                        dist(rng), dist(rng), dist(rng)));
            }
            else
            {
                // default method is to generate inside the bbox
                auto&& bbox = mesh.getBounds();
                for (auto i = 0; i < NUMGEN; i++)
                    inputs.emplace_back(sampleBox(bbox,
                        dist(rng), dist(rng), dist(rng)));
            }
        }

        if (!sradius.size())
            queryManager.query(inputs, qmethod, outputs);
        else
            queryManager.query(inputs, qmethod, outputs, sradius.get().f32);

        // print output
        for (size_t i = 0; i < inputs.size(); i++)
        {
            std::cout << "--------------------------------------" << std::endl;
            std::cout << " { " << std::endl;
            std::cout << "\t Query Pt          : " << inputs[i] << std::endl;
            std::cout << "\t Query Result      : " << outputs[i] << std::endl;
            std::cout << "\t Query Primitives  : " << std::endl << std::endl;
            for (auto&& r : outputs[i].myClosestPoints)
            {
                auto&& triangle = mesh.getPrimitive(r.first);
                std::cout << "\t Id : " << r.first << " Primitive : "
                          << triangle << std::endl;
            }
            std::cout << " } " << std::endl;
            std::cout << "--------------------------------------" << std::endl;
            std::cout << std::endl << std::endl;
        }
    }
    else
    {
        std::cout << "QueryManager init failed." << std::endl;
        return -1;
    }

    return 0;
}