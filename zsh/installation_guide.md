# Comprehensive Zsh Setup Guide

This guide provides step-by-step instructions for installing and configuring Zsh (Z Shell) on a Linux system (specifically tailored for Ubuntu/Debian based distributions). We will cover installation, making Zsh the default shell, installing the "Oh My Zsh" framework, and configuring plugins and themes for a productive workflow.

## 1. Introduction

Zsh is a powerful shell that can operate as both an interactive shell and a scripting language interpreter. It is compatible with Bash but offers significant improvements:
- **Better Auto-completion**: Context-aware completion for files, commands, and arguments.
- **Theming**: Highly customizable prompt themes.
- **Plugins**: A vast ecosystem of plugins to enhance productivity (e.g., git integration, syntax highlighting).

## 2. Prerequisites

Ensure you have `sudo` privileges on your machine.

First, update your package lists:
```bash
sudo apt update
```

## 3. Install Zsh

Install Zsh using the package manager:
```bash
sudo apt install zsh -y
```

Verify the installation by checking the version:
```bash
zsh --version
```

## 4. Make Zsh the Default Shell

Set Zsh as your default shell so it launches automatically when you open a terminal:
```bash
chsh -s $(which zsh)
```
**Note:** You may need to log out and log back in for this change to take effect.

To confirm that Zsh is now your active shell, run:
```bash
echo $SHELL
```
It should output `/bin/zsh` or `/usr/bin/zsh`.

## 5. Install Oh My Zsh

[Oh My Zsh](https://ohmyz.sh/) is an open-source, community-driven framework for managing your Zsh configuration. It comes with thousands of helpful functions, helpers, plugins, and themes.
*(See the [official guide](https://github.com/ohmyzsh/ohmyzsh/wiki/Installing-ZSH) for more details)*.

Install Oh My Zsh via `curl`:
```bash
sh -c "$(curl -fsSL https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh)"
```

## 6. Configuration & Theming

The main configuration file for Zsh is `~/.zshrc`.

### Applying the Recommended Configuration
This repository includes a pre-configured `.zshrc` file tailored for development. It uses the **agnoster** theme and includes several useful plugins.

1.  **Backup your existing config (if any):**
    ```bash
    cp ~/.zshrc ~/.zshrc.backup
    ```

2.  **Copy the configuration from this repository:**
    copy the `.zshrc` file from this directory to your home directory:
    ```bash
    # Assuming you are in the directory containing this guide and the .zshrc file
    cp .zshrc ~/.zshrc
    ```

### Theme: Agnoster
The configuration uses the `af-magic` theme, which provides a informative prompt with Git status integration.

### Enabled Plugins
The recommended configuration enables the following plugins:

| Plugin | Description |
| :--- | :--- |
| `git` | Adds many aliases and functions for Git. |
| `pip` | Autocompletion for pip commands. |
| `python` | Alias shortcuts for python. |
| `ubuntu` | Ubuntu-specific aliases. |
| `sudo` | Pressing `Esc` twice puts `sudo` in front of the current command. |
| `extract` | Powerful plugin to extract any archive file using `x` command (e.g. `x archive.tar.gz`). |
| `poetry` | Autocompletion for Poetry (Python dependency management). |
| `poetry-env` | Integrates Poetry environment activation. |

### key Configuration Settings
The provided `.zshrc` file also includes some user-friendly defaults:

- **Auto-Update**: Oh My Zsh is set to update automatically without prompting (`zstyle ':omz:update' mode auto`).
- **Auto-Correction**: Command auto-correction is enabled (`ENABLE_CORRECTION="true"`). If you make a typo, Zsh will suggest the correct command.
- **Hyphen Insensitivity**: Completion is case-insensitive and treats hyphens/underscores interchangeably (`HYPHEN_INSENSITIVE="true"`), making it easier to tab-complete file names.

## 7. Reload Configuration

Apply the changes by sourcing your `.zshrc` file (or simply restart your terminal):
```bash
source ~/.zshrc
```

## Troubleshooting

- **Fonts not showing correctly?** Ensure your terminal emulator is set to use a Powerline-compatible font (often installed as "Ubuntu Mono" or similar).
- **Plugins missing?** Some plugins might need to be manually installed if they aren't bundled with Oh My Zsh default list. However, standard plugins usually work out of the box.

---
**Enjoy your new supercharged terminal!**