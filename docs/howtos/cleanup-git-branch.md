# Cleanup Git Branch

You may have many local or remote git branches that are no longer needed. 

```bash
git branch -d $(git branch --merged=main | grep -v main)
git fetch --prune
```

* The first command deletes all local branches that have been merged into the main branch, except for the main branch itself. 
* The second command cleans up remote-tracking branches that no longer exist on the remote repository.

## Reference

* https://medium.com/@FlorentDestrema/a-simple-way-to-clean-up-your-git-project-branches-283b87478fbc