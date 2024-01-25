# Example Repository Showing How to Use the *repo* Tools

The *repo_minimal* repository contains a small example of how to use the *repo*
tool suite in your repository.

## Background

Most Omniverse software is built, packaged, and deployed using the *repo* suite
of tools.  The *repo* tools are written in Python so that they are easy to
author, maintain, and use across multiple platforms.

Each Omniverse repository has the following scripts in its root directory:

* *repo* (Linux)
* *repo.bat* (Windows)

The scripts above are all-in-one, bootstrapping scripts that allow for a single
entry point to access the full suite of *repo* tools.  These bootstrapping
scripts are referred to as *repoman*.

Each Omniverse repository can select which tool within the *repo* suite it
wishes to support as follows.

First, the repository specifies the desired *repo* tools it wishes to support in
*deps/repo-deps.packman.xml*. The full list of tools within the *repo* suite can
be found at https://gitlab-master.nvidia.com/omniverse/repo.

Second, the repository configures each tool in *repo.toml* (located in the
repository's root).

Finally, the user can invoke the tool.  To get a list of the *repo* tools
supported by the repository, run:

```bash
./repo --help
```

The remainder of this documents outlines how the purpose of each file in the
repository and how to get the best results out of the *repo* tool suite.

## Required Files

Any *repo*-based repository contains the following *required* files:

**tools/packman/**: Directory containing bootstrapping files for
[packman](https://gitlab-master.nvidia.com/hfannar/packman), Omniverse's package
manager (think: Omnivere's version of [npm](https://www.npmjs.com/)).  Once
these files have been added to the repository, the repository owner need not
edit them.

**tools/repoman/**: Directory containing bootstrapping files for
[repo_man](https://gitlab-master.nvidia.com/omniverse/repo/repo_man).  Once these
files have been added to the repository, the repository owner need not edit
them.

**repo** and **repo.bat**: Platform specific scripts to run the *repo* tools.  The
repository owner should never have to edit these files.

**deps/repo-deps.packman.xml**: A list of *repo* tools to make available to the
repository.  Repository owners will edit this file as new tools are needed.  See
below.

**repo.toml**: Configuration file for *repo* tools.  The repository owner should
edit this file to configure the *repo* tools to meet the repository's needs. See
below.

## Optional Files

**README.md**: This file.  It's helpful, but not needed.

**docs/index.rst**: Used by
[repo_docs](https://gitlab-master.nvidia.com/omniverse/repo/repo_docs) to serve
as the main page of the repository's documentation.

## Quick Start: Using the *repo* Suite in Your Repository

To begin, copy the required files listed above into your repository.

Since this example repository is likely out-of-date, you should next update the
tools:

```bash
# update packman
tools/packman/packman update

# update the repo suite of tools
repo update repo_
```

You're now ready to go!  See the following sections for more information on
adding, configuring, and running tools.

## Adding a Tool

A list of available tools can found at https://gitlab-master.nvidia.com/omniverse/repo.  To find the package containing the tool, search for the tool at http://packman.ov.nvidia.com/.

To add a tool, add an entry to *deps/repo-deps.packman.xml*.  For example, to
add the *repo_docs* tool:

```xml
<project toolsVersion="5.0">
  <!-- required: the core library for repo suite of tools -->
  <dependency name="repo_man" linkPath="../_repo/deps/repo_man">
    <package name="repo_man" version="1.10.1" />
  </dependency>
  <!-- optional: tool for building and publishing great looking documentation -->
  <dependency name="repo_docs" linkPath="../_repo/deps/repo_docs">
    <package name="repo_docs" version="0.10.1" />
  </dependency>
</project>
```

## Configuring a Tool

All tool configuration happens in *repo.toml*.  For example, here's a
configuration for
[repo_docs](https://gitlab-master.nvidia.com/omniverse/repo/repo_docs), a tool
for building documentation:

```toml
[repo_docs]
enabled = true
docs_root = "docs"
project = "repo_minimal"
name = "Minimal repo_man Based Repository"
version = "0.1.0"
copyright_start = 2022

```

## Running a Tool

The list of tools that are installed in the repo can be shown with:

```bash
repo --help
```

To run a tool, for example *repo_docs*:

```bash
repo docs
```

## Further Reading

For help, reach out to [#ct-omni-repoman](https://nvidia.slack.com/archives/CQEBYDM6K).
