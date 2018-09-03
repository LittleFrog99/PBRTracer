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

private:
    T *target;
    T backup;
};

#endif // UTILITY_SCOPED_ASSIGNMENT
