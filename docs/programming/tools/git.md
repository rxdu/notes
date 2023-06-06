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

## Reference

* [1] https://www.designcise.com/web/tutorial/how-to-find-git-commit-id-by-commit-message