protocol EqualsMethod {
    // Notice that the type-keyword  "Self" helps to solve the
    // curriously recurring interface pattern for direct self-reference
    // FIXME-begin
    func equals(other: Self) -> Bool
    // FIXME-end
}

struct my_int : EqualsMethod {
    // FIXME-begin
    func equals(other: my_int) -> Bool {
        return self.value == other.value
    }
    // FIXME-end

    var value: Int;
}

// Returns the index of an element if it exists
// nil otherwise
func findIndex
    // FIXME-begin
    <T: EqualsMethod>
    // FIXME-end
    (valueToFind: T, array:[T]) -> Int? {

        // FIXME-begin
        for (index, value) in array.enumerated() {
            if value.equals(other: valueToFind) {
                return index
            }
     d7e5d250293337c460c48a051ca3ade49892bc31 3f26e286ff24efca122f8b57f349541b6c26a813 Ivan Imbert <ivan.imbert@epita.fr> 1686741733 +0200	pull: fast-forward
                                                                                                                                                                                                    