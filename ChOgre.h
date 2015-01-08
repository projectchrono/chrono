#if defined(_WIN32) && defined(_CHOGRE_BUILD)
#define CHOGRE_DLL_TAG __declspec(dllexport)
#elif defined(_WIN32) && !defined(_CHOGRE_BUILD)
#define CHOGRE_DLL_TAG __declspec(dllimport)
#elif !defined(_WIN32)
#define CHOGRE_DLL_TAG
#endif