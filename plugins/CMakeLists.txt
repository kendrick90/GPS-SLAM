# Plugins CMakeLists.txt
# Each plugin is a separate shared library that implements an interface

# Plugin subdirectories (add as plugins are implemented)
# add_subdirectory(input)
# add_subdirectory(depth)
# add_subdirectory(segmentation)
# add_subdirectory(features)
# add_subdirectory(slam)
# add_subdirectory(rendering)
# add_subdirectory(viewers)

# Note: Plugins are loaded dynamically at runtime.
# To add a new plugin:
# 1. Create a subdirectory in the appropriate category
# 2. Implement the relevant interface (IInputSource, ITracker, IMapper, etc.)
# 3. Use SAP_REGISTER_PLUGIN() macro to register the plugin
# 4. Add to the parent CMakeLists.txt
