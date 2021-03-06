#include "core/api.h"
#include "core/bounds.h"
#include "parallel.h"

static void usage(const char *msg = nullptr) {
    if (msg)
        fprintf(stderr, "PBRTracer: %s\n\n", msg);

    fprintf(stderr, R"(usage: pbr-tacer [<options>] <filename.pbrt...>
Rendering options:
  --help               Print this help text.
  --nthreads <num>     Use specified number of threads for rendering.
  --outfile <filename> Write the final image to the given filename.
  --quick              Automatically reduce a number of quality settings to
                       render more quickly.
  --quiet              Suppress all text output other than error messages.

Logging options:
  --logdir <dir>       Specify directory that log files should be written to.
                       Default: system temp directory (e.g. $TMPDIR or /tmp).
  --logtostderr        Print all logging messages to stderr.
  --minloglevel <num>  Log messages at or above this level (0 -> INFO,
                       1 -> WARNING, 2 -> ERROR, 3-> FATAL). Default: 0.
  --v <verbosity>      Set VLOG verbosity.

Reformatting options:
  --cat                Print a reformatted version of the input file(s) to
                       standard output. Does not render an image.
  --toply              Print a reformatted version of the input file(s) to
                       standard output and convert all triangle meshes to
                       PLY files. Does not render an image.
)");
}

int main(int argc, char *argv[]) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = 1; // Warning and above.

    Options options;
    vector<string> filenames;
    // Process command line argument
    for (int i = 1; i < argc; ++i) {
        if (!strcmp(argv[i], "--nthreads") || !strcmp(argv[i], "-nthreads")) {
            if (i + 1 == argc)
                usage("missing value after --nthreads argument");
            options.nThreads = atoi(argv[++i]);
        } else if (!strncmp(argv[i], "--nthreads=", 11)) {
            options.nThreads = atoi(&argv[i][11]);
        } else if (!strcmp(argv[i], "--outfile") || !strcmp(argv[i], "-outfile")) {
            if (i + 1 == argc)
                usage("missing value after --outfile argument");
            options.imageFile = argv[++i];
        } else if (!strncmp(argv[i], "--outfile=", 10)) {
            options.imageFile = &argv[i][10];
        } else if (!strcmp(argv[i], "--logdir") || !strcmp(argv[i], "-logdir")) {
            if (i + 1 == argc)
                usage("missing value after --logdir argument");
            FLAGS_log_dir = argv[++i];
        } else if (!strncmp(argv[i], "--logdir=", 9)) {
            FLAGS_log_dir = &argv[i][9];
        } else if (!strcmp(argv[i], "--minloglevel") ||
                   !strcmp(argv[i], "-minloglevel")) {
            if (i + 1 == argc)
                usage("missing value after --minloglevel argument");
            FLAGS_minloglevel = atoi(argv[++i]);
        } else if (!strncmp(argv[i], "--minloglevel=", 14)) {
            FLAGS_minloglevel = atoi(&argv[i][14]);
        } else if (!strcmp(argv[i], "--quick") || !strcmp(argv[i], "-quick")) {
            options.quickRender = true;
        } else if (!strcmp(argv[i], "--quiet") || !strcmp(argv[i], "-quiet")) {
            options.quiet = true;
        } else if (!strcmp(argv[i], "--cat") || !strcmp(argv[i], "-cat")) {
            options.cat = true;
        } else if (!strcmp(argv[i], "--toply") || !strcmp(argv[i], "-toply")) {
            options.toPly = true;
        } else if (!strcmp(argv[i], "--v") || !strcmp(argv[i], "-v")) {
            if (i + 1 == argc)
                usage("missing value after --v argument");
            FLAGS_v = atoi(argv[++i]);
        } else if (!strncmp(argv[i], "--v=", 4)) {
          FLAGS_v = atoi(argv[i] + 4);
        }
        else if (!strcmp(argv[i], "--logtostderr")) {
          FLAGS_logtostderr = true;
        } else if (!strcmp(argv[i], "--help") || !strcmp(argv[i], "-help") ||
                   !strcmp(argv[i], "-h")) {
            usage();
            return 0;
        } else
            filenames.push_back(argv[i]);
    }

    // Print welcome banner
    if (!options.quiet && !options.cat && !options.toPly) {
        if (sizeof(void *) == 4)
            printf("*** WARNING: This is a 32-bit build of. It will crash "
                   "if used to render highly complex scenes. ***\n");
        printf("PBRTracer (built %s at %s) [Detected %d cores]\n",
               __DATE__, __TIME__, Parallel::numSystemCores());
        printf( "The source code to this project is covered by the BSD License.\n");
        fflush(stdout);
    }
    API::init(options);
    // Process scene description
    if (filenames.empty()) {
        // Parse scene from standard input
        API::parseFile("-");
    } else {
        // Parse scene from input files
        for (const string &f : filenames)
            API::parseFile(f);
    }
    API::cleanup();
    return 0;
}
