const fs = require("fs");
const { exec, execSync, spawn, spawnSync, fork } = require('child_process');
const path = require("path");

exec("git worktree prune", () => {




    fs.unlink(path.join("..", "alicenode/client/worktreeList.txt"), (err) => {

        if (err) throw err;
        //get the names of current worktrees
        exec("git worktree list --porcelain | grep -e 'worktree' | cut -d ' ' -f 2", (stderr, err) => {

            for (var i = 0; i < (err.toString().split('\n')).length; i++) {
                var worktreeName = ((err.toString().split('\n'))[i].split("alicenode_inhabitat/")[1]);
                if (worktreeName !== undefined) {

                    fs.appendFile(path.join("..", 'alicenode/client/worktreeList.txt'), worktreeName + "\n", function (err) {
                        if (err) {
                            console.log(err)
                            
                        } else {
                            console.log("worktreeList.txt updated")
                        }
                    }) 
                }   
            }
        })

    })
})
