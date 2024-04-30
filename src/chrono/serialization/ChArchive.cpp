
#include "chrono/serialization/ChArchive.h"

namespace chrono {

ChArchive::ChArchive() {
    use_versions = true;
    cluster_class_versions = true;
}

void ChArchive::SetUseVersions(bool muse) {
    this->use_versions = muse;
}

void ChArchive::SetClusterClassVersions(bool mcl) {
    this->cluster_class_versions = mcl;
}

void ChArchiveOut::PutPointer(void* object, bool& already_stored, size_t& obj_ID) {
    if (this->internal_ptr_id.find(static_cast<void*>(object)) != this->internal_ptr_id.end()) {
        already_stored = true;
        obj_ID = internal_ptr_id[static_cast<void*>(object)];
        return;
    }

    // wasn't in list.. add to it
    ++currentID;
    obj_ID = currentID;
    internal_ptr_id[static_cast<void*>(object)] = obj_ID;
    already_stored = false;
    return;
}

void ChArchiveOut::out_version(int mver, const std::type_index mtypeid) {
    if (use_versions) {
        std::string class_name = ChClassFactory::IsClassRegistered(mtypeid) ? ChClassFactory::GetClassTagName(mtypeid)
                                                                            : std::string(mtypeid.name());
        // avoid issues with XML format
        std::replace(class_name.begin(), class_name.end(), '<', '_');
        std::replace(class_name.begin(), class_name.end(), '>', '_');
        std::replace(class_name.begin(), class_name.end(), ' ', '_');
        // avoid issues with FMU variable naming format
        std::replace(class_name.begin(), class_name.end(), ':', '_');
        this->out(ChNameValue<int>(("_version_" + class_name).c_str(), mver));
    }
}

ChArchiveIn::ChArchiveIn() : can_tolerate_missing_tokens(false) {
    internal_ptr_id.clear();
    internal_ptr_id[nullptr] = 0;  // null pointer -> ID=0.
    internal_id_ptr.clear();
    internal_id_ptr[0] = nullptr;  // ID=0 -> null pointer.
}

int ChArchiveIn::in_version(const std::type_index mtypeid) {
    int mver;
    std::string class_name = ChClassFactory::IsClassRegistered(mtypeid) ? ChClassFactory::GetClassTagName(mtypeid)
                                                                        : std::string(mtypeid.name());
    // avoid issues with XML format
    std::replace(class_name.begin(), class_name.end(), '<', '_');
    std::replace(class_name.begin(), class_name.end(), '>', '_');
    std::replace(class_name.begin(), class_name.end(), ' ', '_');
    // avoid issues with FMU variable naming format
    std::replace(class_name.begin(), class_name.end(), ':', '_');
    this->in(ChNameValue<int>(("_version_" + class_name).c_str(), mver));
    return mver;
}

}  // end namespace chrono
