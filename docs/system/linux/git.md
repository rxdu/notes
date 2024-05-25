# Git Reference

## Find Commit ID by Commit Message

* General search

```bash
$ git log --grep=<pattern>
```

* Case-insensitive match 

```bash
$ git log --grep=<pattern> -i
```

* Search multiple keywords

Commit message that matches "pattern1" **or** "pattern2"

```bash
$ git log --grep=<pattern1> --grep=<pattern2>
```

Commit message that matches "pattern1" **and** "pattern2"

```bash
$ git log --grep=<pattern1> --grep=<pattern2> --all-match
```

## Remove a submodule

Remove the filetree at <path-to-submodule>, and the submodule's entry in the .gitmodules file
  
```
$ git rm <path-to-submodule>, and commit.
```

"The .git dir of the submodule is kept around (in the modules/ directory of the main project's .git dir), 'to make it possible to checkout past commits without requiring fetching from another repository'. If you nonetheless want to remove this info, manually delete the submodule's directory in .git/modules/, and remove the submodule's entry in the file .git/config."[2]

```bash
$ rm -rf .git/modules/<path-to-submodule>
$ git config --remove-section submodule.<path-to-submodule>.
```


## Reference

* [1] https://www.designcise.com/web/tutorial/how-to-find-git-commit-id-by-commit-message
* [2] https://stackoverflow.com/a/1260982