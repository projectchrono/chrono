#include "chrono/serialization/ChObjectExplorer.h"

namespace chrono {
ChObjectExplorer::ChObjectExplorer() {
    this->found = false;
    this->searched_property = "";
    this->tablevel = 0;
    this->use_wildcards = true;
    this->use_user_names = true;
    this->find_all = false;
    this->found_obj = false;
    this->SetUseVersions(false);
}
void ChObjectExplorer::out(ChNameValue<unsigned int> bVal) {
    if (this->found && !this->find_all)
        return;
    if (this->tablevel != this->search_tokens.size() - 1)
        return;
    if (this->MatchName(search_tokens[this->tablevel], bVal.name())) {
        this->results.push_back(new ChValueSpecific<unsigned int>(bVal.value(), bVal.name(), bVal.flags(),
                                                                  bVal.GetCausality(), bVal.GetVariability()));
        this->found = true;
    }
}
void ChObjectExplorer::out(ChNameValue<char> bVal) {
    if (this->found && !this->find_all)
        return;
    if (this->tablevel != this->search_tokens.size() - 1)
        return;
    if (this->MatchName(search_tokens[this->tablevel], bVal.name())) {
        this->results.push_back(new ChValueSpecific<char>(bVal.value(), bVal.name(), bVal.flags(), bVal.GetCausality(),
                                                          bVal.GetVariability()));
        this->found = true;
    }
}
void ChObjectExplorer::out(ChNameValue<float> bVal) {
    if (this->found && !this->find_all)
        return;
    if (this->tablevel != this->search_tokens.size() - 1)
        return;
    if (this->MatchName(search_tokens[this->tablevel], bVal.name())) {
        this->results.push_back(new ChValueSpecific<float>(bVal.value(), bVal.name(), bVal.flags(), bVal.GetCausality(),
                                                           bVal.GetVariability()));
        this->found = true;
    }
}
void ChObjectExplorer::out(ChNameValue<double> bVal) {
    if (this->found && !this->find_all)
        return;
    if (this->tablevel != this->search_tokens.size() - 1)
        return;
    if (this->MatchName(search_tokens[this->tablevel], bVal.name())) {
        this->results.push_back(new ChValueSpecific<double>(bVal.value(), bVal.name(), bVal.flags(),
                                                            bVal.GetCausality(), bVal.GetVariability()));
        this->found = true;
    }
}
void ChObjectExplorer::out(ChNameValue<int> bVal) {
    if (this->found && !this->find_all)
        return;
    if (this->tablevel != this->search_tokens.size() - 1)
        return;
    if (this->MatchName(search_tokens[this->tablevel], bVal.name())) {
        this->results.push_back(new ChValueSpecific<int>(bVal.value(), bVal.name(), bVal.flags(), bVal.GetCausality(),
                                                         bVal.GetVariability()));
        this->found = true;
    }
}

std::vector<ChValue*>& ChObjectExplorer::FetchValues(ChValue& root, const std::string& value_name) {
    this->PrepareSearch(value_name);
    this->find_all = true;
    root.CallOut(*this);
    return this->results;
}
void ChObjectExplorer::out(ChNameValue<bool> bVal) {
    if (this->found && !this->find_all)
        return;
    if (this->tablevel != this->search_tokens.size())
        return;
    if (this->MatchName(search_tokens[this->tablevel], bVal.name())) {
        this->results.push_back(new ChValueSpecific<bool>(bVal.value(), bVal.name(), bVal.flags(), bVal.GetCausality(),
                                                          bVal.GetVariability()));
        this->found = true;
    }
}
void ChObjectExplorer::out(ChNameValue<std::string> bVal) {
    if (this->found)
        return;
    if (this->tablevel != this->search_tokens.size() - 1)
        return;
    if (this->MatchName(search_tokens[this->tablevel], bVal.name())) {
        this->results.push_back(new ChValueSpecific<std::string>(bVal.value(), bVal.name(), bVal.flags(),
                                                                 bVal.GetCausality(), bVal.GetVariability()));
        this->found = true;
    }
}
void ChObjectExplorer::out(ChNameValue<unsigned long> bVal) {
    if (this->found && !this->find_all)
        return;
    if (this->tablevel != this->search_tokens.size() - 1)
        return;
    if (this->MatchName(search_tokens[this->tablevel], bVal.name())) {
        this->results.push_back(new ChValueSpecific<unsigned long>(bVal.value(), bVal.name(), bVal.flags(),
                                                                   bVal.GetCausality(), bVal.GetVariability()));
        this->found = true;
    }
}
void ChObjectExplorer::out(ChNameValue<unsigned long long> bVal) {
    if (this->found)
        return;
    if (this->tablevel != this->search_tokens.size() - 1)
        return;
    if (this->MatchName(search_tokens[this->tablevel], bVal.name())) {
        this->results.push_back(new ChValueSpecific<unsigned long long>(bVal.value(), bVal.name(), bVal.flags(),
                                                                        bVal.GetCausality(), bVal.GetVariability()));
        this->found = true;
    }
}
void ChObjectExplorer::out(ChNameValue<ChEnumMapperBase> bVal) {
    std::string mstr = bVal.value().GetValueAsString();
    if (this->found && !this->find_all)
        return;
    if (this->tablevel != this->search_tokens.size() - 1)
        return;
    if (this->MatchName(search_tokens[this->tablevel], bVal.name())) {
        this->results.push_back(new ChValueSpecific<std::string>(mstr, bVal.name(), bVal.flags(), bVal.GetCausality(),
                                                                 bVal.GetVariability()));
        this->found = true;
    }
}
void ChObjectExplorer::out_array_pre(ChValue& bVal, size_t msize) {
    if (this->found && !this->find_all)
        return;
    if (this->tablevel >= this->search_tokens.size())
        return;

    if (this->MatchName(search_tokens[this->tablevel], bVal.name()) || tablevel == 0) {
        this->found_obj = true;

        if (this->tablevel + 1 == this->search_tokens.size()) {
            this->results.push_back(bVal.new_clone());
            this->found = true;
        }
        ++tablevel;
        in_array.push_back(true);
    }
}
void ChObjectExplorer::out_array_between(ChValue& bVal, size_t msize) {}
void ChObjectExplorer::out_array_end(ChValue& bVal, size_t msize) {
    if (in_array[tablevel] == true) {
        --tablevel;
        in_array.pop_back();
    }
}

// for custom c++ objects:

void ChObjectExplorer::out(ChValue& bVal, bool tracked, size_t obj_ID) {
    if (this->found && !this->find_all)
        return;
    if (this->tablevel >= this->search_tokens.size())
        return;

    bool matched = this->MatchName(search_tokens[this->tablevel], bVal.name()) || (tablevel == 0);
    if (!matched && tablevel && this->in_array[tablevel] && this->use_user_names &&
        !search_tokens[this->tablevel].empty() &&
        (search_tokens[this->tablevel].front() == *"'" && search_tokens[this->tablevel].back() == *"'") &&
        bVal.HasArchiveContainerName()) {
        std::string username = search_tokens[this->tablevel].substr(1, search_tokens[this->tablevel].length() - 2);
        matched = this->MatchName(username, bVal.CallArchiveContainerName().c_str());
    }

    if (matched) {
        this->found_obj = true;
        if (this->tablevel + 1 == this->search_tokens.size()) {
            this->results.push_back(bVal.new_clone());
            this->found = true;
        } else {
            ++tablevel;
            in_array.push_back(false);
            bVal.CallArchiveOut(*this);
            --tablevel;
            in_array.pop_back();
        }
    }
}
void ChObjectExplorer::out_ref(ChValue& bVal, bool already_inserted, size_t obj_ID, size_t ext_ID) {
    if (this->found && !this->find_all)
        return;
    if (this->tablevel >= this->search_tokens.size())
        return;

    bool matched = this->MatchName(search_tokens[this->tablevel], bVal.name()) || (tablevel == 0);
    if (!matched && tablevel && this->in_array[tablevel] && this->use_user_names &&
        !search_tokens[this->tablevel].empty() &&
        (search_tokens[this->tablevel].front() == *"'" && search_tokens[this->tablevel].back() == *"'") &&
        bVal.HasArchiveContainerName()) {
        std::string username = search_tokens[this->tablevel].substr(1, search_tokens[this->tablevel].length() - 2);
        matched = this->MatchName(username, bVal.CallArchiveContainerName().c_str());
    }

    if (matched) {
        this->found_obj = true;
        if (this->tablevel + 1 == this->search_tokens.size()) {
            this->results.push_back(bVal.new_clone());
            this->found = true;
        } else {
            ++tablevel;
            in_array.push_back(false);
            bVal.CallArchiveOut(*this);
            --tablevel;
            in_array.pop_back();
        }
    }
}

// There should be no need to call this by hand, as it is invoked
// automatically at the beginning of each search.

void ChObjectExplorer::ClearSearch() {
    this->found = false;
    this->find_all = false;
    this->found_obj = false;
    this->tablevel = 0;
    this->results.clear();
    this->search_tokens.clear();
    this->search_tokens.push_back("");  // for root lavel
    this->in_array.clear();
    this->in_array.push_back(false);  // for root level
    for (int i = 0; i < this->results.size(); ++i) {
        delete (this->results[i]);
    }
}
void ChObjectExplorer::PrepareSearch(const std::string& msearched_property) {
    ClearSearch();
    this->searched_property = msearched_property;
    std::stringstream test(msearched_property);
    std::string segment;

    while (std::getline(test, segment, '/')) {
        search_tokens.push_back(segment);
    }
}
int ChObjectExplorer::wildcard_compare(const char* wildcard, const char* string) {
    const char *cp = 0, *mp = 0;

    while ((*string) && (*wildcard != '*')) {
        if ((*wildcard != *string) && (*wildcard != '?')) {
            return 0;
        }
        wildcard++;
        string++;
    }

    while (*string) {
        if (*wildcard == '*') {
            if (!*++wildcard) {
                return 1;
            }
            mp = wildcard;
            cp = string + 1;
        } else {
            if ((*wildcard == *string) || (*wildcard == '?')) {
                wildcard++;
                string++;
            } else {
                wildcard = mp;
                string = cp++;
            }
        }
    }

    while (*wildcard == '*') {
        wildcard++;
    }
    return !*wildcard;
}
bool ChObjectExplorer::MatchName(const std::string& token, const std::string& string) {
    if (use_wildcards) {
        if (wildcard_compare(token.c_str(), string.c_str()))
            return true;
    } else {
        if (token == string)
            return true;
    }

    return false;
}
}  // end namespace chrono
