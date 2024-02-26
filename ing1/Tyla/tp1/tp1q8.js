// FIXME-begin
function mycurry(f) {
    return function (...args) {
        if (args.length >= f.length) {
            return f(...args);
        } else {
            return function (...args2) {
                return mycurry(f)(...args, ...args2);
            }
        }
    }
}
function build_sentence(p1, p2, p3) {
    return p1 + ' (buddy of ' + p2 + ')' + p3;
}
// FIXME-end

const curried = mycurry(build_sentence);
console.log(curried("Tigrou")("Spider")(" was here!"))
console.log(curried("Spider")("Tigrou")(" was also here!"))
console.log(curried("Tigrou", "Spider", " are in a boat... "))
console.log(curried(1)(2, 3))
