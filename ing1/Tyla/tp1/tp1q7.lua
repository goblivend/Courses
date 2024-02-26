-- FIXME-begin
function powerize(n)
  local i = 0
  return function()
    local res = n ^ i
    i = i + 1
    return math.floor(res)
  end
end
-- FIXME-end

c1 = powerize(1)
print(c1(), c1(), c1(), c1(), c1())
c2 = powerize(2)
print(c2(), c2(), c2(), c2(), c2())
c3 = powerize(3)
print(c3(), c3(), c3(), c3(), c3())
