"""Test batch mode argument validation without connecting to device."""
import sys
from pathlib import Path
import capture_collection as cc

def test_batch_home():
    """Test batch mode for P0."""
    print("测试1: 批量模式 P0")
    args = ['batch', '--home', '--port', 'COM11', '--holder', 'claw', '--grip-target', '1800']
    try:
        # This will fail at device lock check, but validates arguments
        cc.main(args)
    except SystemExit as e:
        print(f"  退出码: {e.code}")
    except Exception as e:
        if '设备锁' in str(e) or 'lease' in str(e).lower():
            print("  ✓ 参数验证通过（预期在设备锁检查处失败）")
        else:
            print(f"  ✗ 意外错误: {e}")

def test_batch_route():
    """Test batch mode for a route."""
    print("\n测试2: 批量模式 红→P1")
    args = ['batch', '--color', '红', '--target', 'P1', '--port', 'COM11',
            '--holder', 'claw', '--grip-open', '1800', '--grip-close', '2000']
    try:
        cc.main(args)
    except SystemExit as e:
        print(f"  退出码: {e.code}")
    except Exception as e:
        if '设备锁' in str(e) or 'lease' in str(e).lower():
            print("  ✓ 参数验证通过（预期在设备锁检查处失败）")
        else:
            print(f"  ✗ 意外错误: {e}")

def test_batch_missing_args():
    """Test batch mode with missing required arguments."""
    print("\n测试3: 缺少必需参数")
    args = ['batch', '--color', '红', '--target', 'P1', '--port', 'COM11',
            '--holder', 'claw']  # Missing grip-open and grip-close
    try:
        cc.main(args)
    except SystemExit as e:
        if e.code != 0:
            print("  ✓ 正确拒绝（缺少夹爪目标）")
        else:
            print("  ✗ 应该拒绝但通过了")
    except Exception as e:
        if 'grip' in str(e).lower():
            print("  ✓ 正确拒绝（缺少夹爪目标）")
        else:
            print(f"  ✗ 意外错误: {e}")

if __name__ == '__main__':
    print("批量模式离线测试\n" + "="*50)
    test_batch_home()
    test_batch_route()
    test_batch_missing_args()
    print("\n" + "="*50)
    print("测试完成")
