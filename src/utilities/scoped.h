#ifndef UTILITY_SCOPED_ASSIGNMENT
#define UTILITY_SCOPED_ASSIGNMENT

template <class T>
class ScopedAssignment {
public:
    ScopedAssignment(T *target = nullptr, T value = T()) : target(target) {
        if (target) {
            backup = *target;
            *target = value;
        }
    }

    ~ScopedAssignment() { if (target) *target = backup; }

    ScopedAssignment(const ScopedAssignment &) = delete;
    ScopedAssignment & operator = (const ScopedAssignment &) = delete;

    ScopedAssignment & operator = (ScopedAssignment &&other) {
        target = other.target;
        backup = other.backup;
        other.target = nullptr;
        return *this;
    }

private:
    T *target, backup;
};

#endif // UTILITY_SCOPED_ASSIGNMENT
