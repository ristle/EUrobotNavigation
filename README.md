## Welcome to GitHub Pages

You can use the [editor on GitHub](https://github.com/ristle/EUrobotNavigation/edit/master/README.md) to maintain and preview the content for your website in Markdown files.

Whenever you commit to this repository, GitHub Pages will run [Jekyll](https://jekyllrb.com/) to rebuild the pages in your site, from the content in your Markdown files.

### Markdown

Markdown is a lightweight and easy-to-use syntax for styling your writing. It includes conventions for

```markdown
Syntax highlighted code block

# Header 1
## Header 2
### Header 3

- Bulleted
- List

1. Numbered
2. List

**Bold** and _Italic_ and `Code` text

[Link](url) and ![Image](src)
```

For more details see [GitHub Flavored Markdown](https://guides.github.com/features/mastering-markdown/).

### Jekyll Themes

Your Pages site will use the layout and styles from the Jekyll theme you have selected in your [repository settings](https://github.com/ristle/EUrobotNavigation/settings). The name of this theme is saved in the Jekyll `_config.yml` configuration file.

### Support or Contact

Having trouble with Pages? Check out our [documentation](https://help.github.com/categories/github-pages-basics/) or [contact support](https://github.com/contact) and we’ll help you sort it out.
## Основной стек
	Это проект написанный внутри лаборатории Робототехники Сберанка стек. Основу : global_planner - Саша Гамаюнов, сотрудник Лаборатории, как и первую итерацию trajectory regulator. Остальное стажер Лаборатории - как создание local_planner, так и доработку cost_map_server вместе с trajectory regulator.

	# Архитектура 
	Из топиков располложения противников, которые настраиваются в параметрах проекта cost_map_server, берется информация для последующего нанесения на карту, написаннкую https://github.com/stonier/cost_map. Карта одна и для global, и для local планнера. Там нанесена инфляция, которая важна при постройке маршрута, алгоритом Theta* ( global planner ) и методом потенциальных полей( local planner ).

