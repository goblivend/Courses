// FIXME-begin
function sumofall(p1 = 0, p2 = 0, ...RestParameters) {
    let sum = p1 + p2;
    for (let i = 0; i < RestParameters.length; i++) {
        sum += RestParameters[i];
    }
    return sum;
}
// FIXME-end


console.log(sumofall());
console.log(sumofall(1));
console.log(sumofall(1, 2));
console.log(sumofall(p1 = 9));
console.log(sumofall(p2 = 8));
console.log(sumofall(p2 = 8, p1 = 14));
console.log(sumofall(1, 2, 3));
