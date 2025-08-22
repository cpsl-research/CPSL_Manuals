Here’s a ready-to-copy README you can paste directly into a file like `README.md`:

# Tmux Setup with TPM (Tmux Plugin Manager)

This guide explains how to install **tmux**, set up the **Tmux Plugin Manager (TPM)**, create a `.tmux.conf` file, and include a basic set of plugins.

---

## 1. Install tmux

On Ubuntu/Debian:
```bash
sudo apt update
sudo apt install tmux
````

Verify installation:

```bash
tmux -V
```

You should see the version, e.g., `tmux 3.4`.

---

## 2. Install TPM (Tmux Plugin Manager)

Clone the TPM repository into the tmux plugins folder:

```bash
git clone https://github.com/tmux-plugins/tpm ~/.tmux/plugins/tpm
```

---

## 3. Create a `.tmux.conf` file

Create the config file in your home directory:

```bash
nano ~/.tmux.conf
```

Paste the following contents:

```tmux
# Other settings
set -g mouse on

# List of TPM plugins
set -g @plugin 'tmux-plugins/tpm'
set -g @plugin 'tmux-plugins/tmux-sensible'
set -g @plugin 'tmux-plugins/tmux-cpu'
set -g @plugin 'jaclu/tmux-menus'      # for easier menus
set -g @plugin 'tmux-plugins/tmux-sidebar'  # for viewing files
set -g @plugin 'tmux-plugins/tmux-yank'    # for easier copying text

# Other plugins can be added here...

# Initialize other plugin-specific settings
set -g @menus_trigger 'Space'  # setup menus

# Initialize TPM
run '~/.tmux/plugins/tpm/tpm'
```

Save and exit (`Ctrl+O`, `Enter`, `Ctrl+X` in nano).

---

## 4. Start tmux and install plugins

1. Launch tmux:

```bash
tmux
```

2. Press **`<prefix> + I`** to install plugins.

   * Default prefix is **Ctrl+b**.
   * So press **Ctrl+b**, release, then **Shift+i**.

3. TPM will download and install all listed plugins.

---

## 5. Reload configuration

If you update `.tmux.conf`, reload without restarting tmux:

```bash
tmux source-file ~/.tmux.conf
```

---

### 6. Useful tips

* **Mouse support:** Click between panes and windows.
* **tmux-yank:** Copy text directly to your system clipboard.
* **tmux-menus:** Press your defined trigger (Space) to open plugin menus.
* **tmux-sidebar:** Navigate files in a side pane.