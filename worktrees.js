const { exec, execSync, spawn, spawnSync, fork } = require('child_process');

exec("git worktree list --porcelain", (stderr, err) => {
    //	console.log(err)
        test = (err.replace("worktree ", ""))
///////
//////
/////
////
///
//TODO: the output from the git worktree list --porcelain is in an array, so its not retrieving the correct names

        //if (err.includes == "worktree"){
            //var worktreeName = .substr(err.lastIndexOf('/') + 1);

            if (test.includes("worktree")) {

                console.log(test)

            }
        //     wName = (worktreeName.replace("\n\n", ""))
        //     console.log("\n\n\n\n\n" + wName + "\n\n\n")
        //    // fs.appendFile(path.join("..", 'alicenode/client/worktreeList.txt'), wName + "\n", function (err) {
        //         if (err) {
        //             console.log(err)
                    
        //         } else {
        //             //worktreeList.txt updated
        //         }
            })



// exec("git worktree list", (stderr, err) => {

//     dave = err.split("[")

//     test = dave.substr(0, dave.indexOf(','))
//     console.log(test)
//     // var lines = err.split('worktree');
    
//     // console.log(lines)
//     // for(var line = 0; line < lines.length; line++){
//     //     //console.log(lines[line])

//     //     trees = (lines[line]).split("worktree")
//     //     console.log(trees)
//    // }
// })