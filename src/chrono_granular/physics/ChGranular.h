#pragma once

namespace chrono {
    /// Global data manager for Chrono::Granular.
    class CH_GRANULAR_API ChGranularDataManager {
    public:
        ChGranularDataManager();
        ~ChGranularDataManager();

        /// Structure that contains the data on the host, the naming convention is
        /// from when the code supported the GPU (host vs device).
        host_container host_data;
        shape_container shape_data;


    };
}