# FIXME-begin
class Foo:
    def __init__(self):
        self.i = 0

    def increment(self):
        self.i += 1
        return self
# FIXME-end

values = [Foo(),Foo(), Foo(), Foo()]
print([x.i for x in values])

values[0].increment()
values[0].increment()


tmp = [
    # FIXME-begin
    values[i].increment() for i in range(len(values))
    # FIXME-end
]

print([x.i for x in tmp])
tmp[1].i = 12
print([x.i for x in values])
print([x.i for x in tmp])
