"""Offline progress/validation of canonical files; no serial operations."""
import argparse
import sys
import collection as c


def check_progress(root=c.ROOT):
    total, routes_complete, errors, configuration = 0, 0, [], None
    for path, color, target in c.paths(root):
        name = '初始位P0' if color is None else f'{color}→{target}'
        expected = 1 if color is None else 8
        try:
            if not path.exists():
                print(f'{name}: 0/{expected}（未初始化）')
                continue
            doc = c.read(path)
            n = c.validate(doc, root, color, target)
            if n and doc['configuration'] is not None:
                if configuration is None:
                    configuration = doc['configuration']
                c.require(configuration == doc['configuration'], '跨路线设备配置不一致')
            total += n
            if color is not None and n == expected:
                routes_complete += 1
            print(f'{name}: {n}/{expected}（仅数据检查）')
        except (ValueError, OSError, TypeError, KeyError) as exc:
            errors.append(f'{name}: {exc}')
            print(f'{name}: 无效 — {exc}')
    print(f'有效采集点: {total}/97；完整路线: {routes_complete}/12；不代表运动验收通过')
    return total, errors


def verify_path(color=None, target=None, root=c.ROOT):
    try:
        doc = c.read(c.route_path(root, color, target))
        c.validate(doc, root, color, target, complete=True)
        for path, other_color, other_target in c.paths(root):
            if path.exists():
                other = c.read(path)
                c.validate(other, root, other_color, other_target)
                if (other['points'] and other['configuration'] is not None
                        and doc['configuration'] is not None):
                    c.require(other['configuration'] == doc['configuration'], '跨路线设备配置不一致')
        print('数据校验通过；单路线、衔接段及组合实测仍须独立验收')
        return True
    except (ValueError, OSError, TypeError, KeyError) as exc:
        print(f'校验失败: {exc}')
        return False


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--root', type=c.Path, default=c.ROOT)
    group = ap.add_mutually_exclusive_group()
    group.add_argument('--verify', nargs=2, metavar=('颜色', '目标'))
    group.add_argument('--verify-home', action='store_true')
    group.add_argument('--verify-all', action='store_true')
    a = ap.parse_args(argv)
    if a.verify or a.verify_home:
        return 0 if verify_path(*(a.verify or (None, None)), root=a.root) else 1
    total, errors = check_progress(a.root)
    return 1 if errors or (a.verify_all and total != 97) else 0


if __name__ == '__main__':
    sys.exit(main())
