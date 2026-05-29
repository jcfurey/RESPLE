// Thin entry point for the Mapping executable. The node implementation lives in
// Mapping.cpp, compiled once into libresple; this wrapper just forwards to its
// exported entry function so the heavy source isn't compiled a second time.
int mappingMain(int argc, char **argv);

int main(int argc, char **argv)
{
    return mappingMain(argc, argv);
}
