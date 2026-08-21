#pragma once

#include <cstdint>
#include <memory>
#include <mutex>

namespace ui_bridge {

template <class T> class Slot {
public:
    void store(std::shared_ptr<const T> v) noexcept {
        std::lock_guard lk(m_);
        slot_ = std::move(v);
        ++generation_;
    }

    std::shared_ptr<const T> load_if_newer(uint64_t &seen) const noexcept {
        std::lock_guard lk(m_);
        if (generation_ == seen)
            return nullptr;
        seen = generation_;
        return slot_;
    }

private:
    mutable std::mutex m_;
    std::shared_ptr<const T> slot_;
    uint64_t generation_ = 0;
};

} // namespace ui_bridge
