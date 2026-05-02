# Bash tab completion for this project's Makefile targets.
#
# One-time setup — add to ~/.bashrc:
#   source /path/to/simple_umbreon_zephyr/completions.bash
#
# Or load just for the current shell:
#   source ./completions.bash

_umbreon_make_complete() {
    local cur="${COMP_WORDS[COMP_CWORD]}"
    local prev="${COMP_WORDS[COMP_CWORD-1]}"
    local makefile
    makefile="$(dirname "${BASH_SOURCE[0]}")/Makefile"

    # Complete targets
    if [[ "${COMP_CWORD}" -eq 1 || "$prev" != *=* ]]; then
        local targets
        targets=$(grep -oP '^[a-zA-Z_-]+(?=:[^=])' "$makefile" 2>/dev/null | grep -v '^\.PHONY')
        COMPREPLY=($(compgen -W "$targets" -- "$cur"))
        return
    fi

    # Complete VAR=value overrides
    if [[ "$cur" == *=* ]]; then
        COMPREPLY=()
        return
    fi

    local vars
    vars=$(grep -oP '^[A-Z_]+(?=\s*\?\s*=)' "$makefile" 2>/dev/null)
    COMPREPLY=($(compgen -W "$vars" -- "$cur") $(compgen -W "$targets" -- "$cur"))
}

complete -F _umbreon_make_complete make
