#!/usr/bin/bash
git add __init__.py
git commit -m "x"
git push
mycroft-msm remove mkz-skill
mycroft-msm install https://github.com/pachinco/mkz-skill.git
