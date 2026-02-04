# BILLEE_software_2026
ROS packages developed for BILLEE

## Using Git Submodules to Freeze Dependencies

This repo now uses submodules for external ROS packages and are stored in the `.gitmodules` file

Git submodules let you include another repository inside your repo and **pin it to a specific commit**.

### 1. Adding a Submodule

```bash
git submodule add https://github.com/ORIGINAL_OWNER/foo.git libs/foo
```

> By default, Git does **not track a branch**. It tracks a **specific commit hash**, which is stored in your main repo. Anyone cloning the repo will get **exactly that commit**, unless they explicitly update it.

---

### 2. Freezing the Submodule at a Commit

To pin the submodule to a specific commit:

```bash
cd libs/foo
git checkout <commit-hash>
cd ../..
git status  # shows submodule has a new commit
git commit -m "Pin foo submodule to commit <hash>"
```
---

### 3. Pin to a Branch but Freeze Updates

You can track a branch but still stay frozen at a specific commit:

```bash
cd libs/foo
git checkout main
cd ../..
git add libs/foo
git commit -m "Track main branch of foo submodule, frozen at current commit"
```


### 4. Cloning with Submodules

After cloning the repo, make sure to initialize submodules:

```bash
git clone --recurse-submodules <repo-url>
# OR
git submodule update --init --recursive
```

### 5. Detached HEAD in Submodules

Git automatically puts submodules in a **detached HEAD** pointing to the pinned commit:

```bash
cd libs/foo
git status
```

## Safely Updating Submodules

### 1. Check the Current Submodule Status

```bash
git submodule status
```

* Shows the current commit for each submodule
* A `-` at the start indicates the submodule isn’t initialized

---

### 2. Fetch and Checkout the Desired Version

```bash
cd libs/foo
git fetch origin
git checkout <commit-or-branch>
```

* Use a **specific commit** to freeze
* Use a **branch** if you want to occasionally track updates

---

### 3. Commit the Updated Submodule

```bash
cd ../..
git add libs/foo
git commit -m "Update foo submodule to <commit-or-branch>"
```

* Updates the main repo to point to the new submodule commit

---

### 4. Pulling Updates Safely

```bash
git pull
git submodule update --init --recursive
```

* Ensures the **exact pinned commits** are checked out
* No unexpected changes from the submodule repo
